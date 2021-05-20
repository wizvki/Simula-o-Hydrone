        
%% calcula atuação nas helices
function actuator(this, dwF, dv, dw)
   
    %% rotacao de equilibrio
    % calculado aq ->
    % <matlab:matlab.desktop.editor.openDocument(which('quadHibridoproposta.m')).goToFunction('equilibrium') 
    % equilibrium(this)>
    wh = this.equilibrium(); 
    
    %% calcula as velocidades desejadas
    if isAir(this.env)
        dv = 0;  
        % * linha = motor;
        % * coluna = quais motores são acionados quando
        %realizam o movimento correspondente para helice aerea
        %
        %       z  xy roll pitch  yaw
        D = [   1   0    0   -1    1;
                1   0    1    0   -1;
                1   0    0    1    1;
                1   0   -1    0   -1  ];
    else
        % * linha = motor;
        % * coluna = quais motores são acionados quando
        %realizam o movimento correspondente para helice aquatica
        %       z  xy roll pitch  yaw
        D = [   1   0   0   -1    0;
                0   1   0    0   -1;
                1   0   0    1    0;
                0   1   0    0    1];
    end
    
    %% ação de controle
    % * w = velocidade desejada 
    % * wh = velocidade para rotação de equilíbrio calculado por aq -> 
    % <matlab:matlab.desktop.editor.openDocument(which('quadHibridoproposta.m')).goToFunction('equilibrium')
    % equilibrium(this)>
    % * dwF = acao de controle de altitude calculado 
    % <matlab:matlab.desktop.editor.openDocument(which('quadHibridoproposta.m')).goToFunction('controllerPos')
    % controllerPos(this, ref)>
    % * dv = velocidade linear calculado aq -> 
    % <matlab:matlab.desktop.editor.openDocument(which('quadHibridoproposta.m')).goToFunction('controllerPos')
    % controllerPos(this, ref)>
    % * dw = velocidade angular; calculado aq -> 
    % <matlab:matlab.desktop.editor.openDocument(which('quadHibridoproposta.m')).goToFunction('controllerAtt')
    % controllerAtt(this, rref)>
    w = [(wh+dwF); dv; dw];
        
    %% velocidades desejadas para os motores
    % aplica as velocidades desejadas para os motores
    % correspondentes ao movimento
    wdes = D*w;
    
    %% atualiza o estado dos motores
    % a função update(this, wdes, dt, env) com entradas 
    % (velocidade desejada nos motores correspondentes, 
    % frequencia interna e tipo de ambiente) está na função aqui -> 
    % <matlab:matlab.desktop.editor.openDocument(which('prop_air_water.m')).goToFunction('update')
    % update(this, wdes, dt, env)>
    %
    % retorna o estado dos motores em relação ao ambiente (como
    % por exemplo, se esta no ar aciona os motores aéreas e
    % não aciona os motores aquáticos e vice versa nesse caso)
    %
    % Acesse o codigo da função aqui -> 
    % <matlab:edit(fullfile('prop_air_water.m')) prop_air_water.m> 
    %
    % Veja  <matlab:doc('prop_air_water') aqui> a janela de HELP do
    % codigo.
    for i = 1:4
        this.mot{i}.update(wdes(i), this.dt, this.env);
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% rotação dos motores no ponto de equilíbrio (tipo hover)
function wh = equilibrium(this)
    
    % medições de ângulos
    [phi, the] = feval(@(a)a{:}, num2cell(this.x(4:5)));
    
    %% efeito da inclinação
    % os ângulos são referentes a diagonal da matriz de rotação
    % angular (eu acho n lembro) aq->
    % <matlab:matlab.desktop.editor.openDocument(which('quadHibridoproposta.m')).goToFunction('B')
    % B(this)>
    if isAir(this.env)
        alpha = cos(phi)*cos(the);
    else
        alpha = cos(the);
    end
    % evita divisão por zero!
    if alpha < 0.1
        alpha = 0.1;
    end
    
    %% gravidade
    % $$ g = |m - \rho  vol| \cdot ||g|| $$ 
    % 
    %gravidade = módulo (massa - densidade do ambiente * volume) * norma matricial da gravidade 
    grav_term = abs(this.m - this.getRho()*this.vol)*norm(this.g, 2);
    
    %% ganho do motor
    % é calculada (ou só definida) coeficiente referente ao motor (ou helice?) aqui: 
    %
    % * se esta acionado os motores aereos ->
    % <matlab:matlab.desktop.editor.openDocument(which('prop_air.m')).goToFunction('getKf')
    % getKf(this)>
    % * se esta acionado os motores aquaticos ->
    % <matlab:matlab.desktop.editor.openDocument(which('prop_water.m')).goToFunction('getKf')
    % getKf(this)>
    % 
    % Acesse o codigo da função aqui -> 
    % <matlab:edit(fullfile('prop_air.m')) prop_air.m> ou 
    % <matlab:edit(fullfile('prop_water.m')) prop_water.m> 
    %
    % Veja  <matlab:doc('prop_air') prop_air.m> ou <matlab:doc('prop_water')
    % prop_water.m> a janela de HELP do codigo.
    kf = this.mot{1}.getKf(); 
    
    %% velocidade para rotação de equilíbrio
    %
    % * para ar:  $wh = \frac{\sqrt{\frac{g}{4 \rho kf}}}{\alpha}$  
    % * para agua:  $wh = \frac{\sqrt{\frac{g}{2 \rho kf}}}{\alpha}$
    %
    % velocidade = sqrt((gravidade)/(4*densidade do
    % ambiente*coeficiente do motor))/angulo do efeito de
    % inclinação 
    if isAir(this.env)
        wh =  sqrt(abs(grav_term)/(4*this.getRho()*kf))/alpha;
    else
        wh = -sqrt(abs(grav_term)/(2*this.getRho()*kf))/alpha;
    end
end

%% calcula aceleracao baseado no modelo
function [at, aa] = getAccel(this, v, q, r)
    
    %% variáveis do ambiente
    % é calculada (ou só definida) densidade do ambiente de acordo
    % com o meio que se encontra aqui:
    % * se esta acionado os motores aéreos ->
    % <matlab:matlab.desktop.editor.openDocument(which('prop_air.m')).goToFunction('getRho')
    % getRho(this)>
    % * se esta acionado os motores aquaticos ->
    % <matlab:matlab.desktop.editor.openDocument(which('prop_water.m')).goToFunction('getRho')
    % getRho(this)>
    % 
    rho = this.getRho();
    
    % Massa Adicional
    added_mass = ((4/6)*pi*(this.rc^3))*rho;
    % inercia adicionais
    added_inertia = (2/5)*(added_mass)*(this.rc^2);
    
    %% Posição %%
    if isAir(this.env)
        % helices superiores nao sao direcionadas
        f(:,1) = this.R()*[0; 0; sum(this.F)];         
    else
        f(:,1) = this.R()*[-sum(this.F([2 4])); 0; sum(this.F([1 3]))];         
    end
    
    %% Gravidade
    % $$f_2 = - m g$$
    f(:,2) = -this.m*this.g; 
    
    %% Arquimedes (empuxo)
    % $$f_3 = \rho vol g$$
    f(:,3) = rho*this.vol*this.g;          
    
    %% Arrasto
    % $$ f_4 = - \frac{1}{2}\rho C_p v |v|$$
    f(:,4) = -(1/2)*rho*this.Cp*v.*abs(v); 
    
    %% Coriolis
    % $$f_5 = - m v \times q$$
    f(:,5) = -cross(q, this.m*v);           
    
    %% Aceleração translacional
    % $$a_t = \frac{\sum_{i=1}^5 f_i}{m + m_a}$$
    at = sum(f,2)/(this.m + added_mass);
    
    %% Atitude %%
    % $$P = mg$$
    %
    % $$E = \rho V g$$
    peso = norm(this.m*this.g, 2);
    empuxo = norm(rho*this.vol*this.g, 2);
    
    % distância entre cg e empuxo
    dist = 0.02; 
    
    %% motores
    % verifica o ambiente que o veículo está para determinar
    % quais motores são considerados em cada eixo do sistema
    if isAir(this.env)
        %se está no ambiente aéreo 
        m(:,1) = [  this.l*(this.F(2)-this.F(4))
                    this.l*(this.F(3)-this.F(1))
                    this.M(1)-this.M(2)+this.M(3)-this.M(4) ];  
    else
        %senão, está no ambiente aquático
        m(:,1) = [  0
                    this.M(3)-this.M(1)
                    -this.M(2)+this.M(4) ];  
    end
    
    %% coriolis
    % $$F_coriolis = - \omega \times I\omega$$
    m(:,2) = -cross(q, this.I*q); 
    
    %% arrasto
    % $$ F_arrasto = -\frac{1}{2}\rho C_r \omega \abs{\omega} $$
    m(:,3) = -(1/2)*rho*this.Cr*q.*abs(q); 
    
    %% momento restaurador (passivo)
    if ~isAir(this.env)
        m(:,4) = -[ dist*sin(r(1))*(peso+empuxo)
                    dist*sin(r(2))*(peso+empuxo)
                    0];                                         
    end
    
    %% calcula a aceleração angular
    % $$a_angular = \frac{I + I_adicional}{\sum F}$$
    %
    aa = (this.I + added_inertia)\sum(m,2);
end

%% modelo dinamico do sistema
function dynamics(this)
    
    % atualiza o estado dos motores
    for i = 1:4
        this.F(i) = this.mot{i}.getForce();
        this.M(i) = this.mot{i}.getMoment();
    end
    
    %% variaveis de estado
    % p: posição x, y, z
    % r: orientação phi, theta, psi (ou pitch roll yaw)
    % v: velocidade linear v_x, v_y, v_z
    % q: velocidade angular \omega_x, \omega_y, \omega_z
    p = this.x(1:3);
    r = this.x(4:6);
    v = this.x(7:9);
    q = this.x(10:12);
    
    %% Runge-Kutta
    % integração da equação de aceleração obtida em
    % <matlab:matlab.desktop.editor.openDocument(which('quadHibridoproposta.m')).goToFunction('getAccel')
    % getAccel(this, v, q, r)>
    % utilizando Meétodo de Runge-Kutta de quarta ordem
    v1 = v;
    q1 = q;
    r1 = r;
    [at1, aa1] = this.getAccel(v1, q1, r1);
    
    v2 = v + 0.5*at1*this.dt;
    q2 = q + 0.5*aa1*this.dt;
    r2 = this.satAngles(r + 0.5*q1*this.dt);
    [at2, aa2] = this.getAccel(v2, q2, r2);
    
    v3 = v + 0.5*at2*this.dt;
    q3 = q + 0.5*aa2*this.dt;
    r3 = this.satAngles(r + 0.5*q2*this.dt);
    [at3, aa3] = this.getAccel(v3, q3, r3);

    v4 = v + at3*this.dt;
    q4 = q + aa3*this.dt;
    r4 = this.satAngles(r + 0.5*q3*this.dt);
    [at4, aa4] = this.getAccel(v4, q4, r4);
    
    %% velocidade translacional
    v = v + (at1 + 2*at2 + 2*at3 + at4)*(this.dt/6.0);
    %% posicao
    p = p + (v1 + 2*v2 + 2*v3 + v4)*(this.dt/6.0);
    %% velocidade rotacional
    q = q + (aa1 + 2*aa2 + 2*aa3 + aa4)*(this.dt/6.0);
    %% angulos
    r = r + (this.B()) \ ((q1 + 2*q2 + 2*q3 + q4)*(this.dt/6.0));
    %% satura angulos para evitar singularidades em this.R()
    r = this.satAngles(r);
    
    %% update estados
    this.x(1:3)   = p;
    this.x(4:6)   = r;
    this.x(7:9)   = v;
    this.x(10:12) = q;
    
    %% incrementa relogio interno
    this.t = this.t + this.dt;
    
    %% atualiza ambiente
    this.setEnvironment();
    
    %% update saidas
    for i = 1:4
        this.u(i) = this.mot{i}.getSpeed();
    end
end
%% atualiza a variavel de ambiente
function setEnvironment(this)
    %se a variavel z>0 é ar
    %senão é água
    if this.x(3) >= 0
        this.env = 'air';
    else
        this.env = 'water';
    end
end
%% matriz de rotacao
function Rzyx = R(this)
    %% Matriz rotacional em torno do eixo X
    % R_x = [1 0 0 \\
    %        0 cos(phi) -sin(phi)\\
    %        0 sin(phi) cos(phi)] 
    Rx = rotx(this.x(4));
    %% Matriz rotacional em torno do eixo X
    % R_y = [cos(theta)  0  sin(theta)\\
    %                 0  1          0 \\
    %        -sin(theta) 0  cos(theta)] 
    Ry = roty(this.x(5));
    %% Matriz rotacional em torno do eixo X
    % R_z = [cos(psi) -sin(psi)  0 \\
    %        sin(psi)  cos(psi)  0\\
    %              0          0  1] 
    Rz = rotz(this.x(6));
    %% Matriz rotacional ZYX
    % R 
    Rzyx = Rz*Ry*Rx;
end
%% satura angulos para evitar singularidades
function ang = satAngles(this, ang)
    ang(1:2) = this.satRot.evaluate(ang(1:2));
    ang(3) = rem(ang(3), 2*pi);
end

%% Matriz rotacional para velocidade angular
function Bot = B(this)
    cphi = cos(this.x(4));
    sphi = sin(this.x(4));
    cthe = cos(this.x(5));
    sthe = sin(this.x(5));
    % R_z = [cos(theta)  0  -cos(phi)sin(theta) \\
    %               0    1             sin(phi) \\
    %        sin(theta)  0  cos(phi)cos(theta)] 
    r1 = cthe;  r2 = 0; r3 = -cphi*sthe;
    r4 = 0;     r5 = 1; r6 = sphi;
    r7 = sthe;  r8 = 0; r9 = cphi*cthe;
    
    Bot = [r1 r2 r3; r4 r5 r6; r7 r8 r9];
end
