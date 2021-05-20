        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        m = (.9)*1.42887;        % [kg] uav mass
        % momentos de inercia
        I = [   144648.98   2041.46     -7870.33
                2041.46     288179.61   -1157.20
                -7870.33    -1157.20    154104.84]*1e-7;
        %
        vol = (1.1)*1.42887e-03;  % Volume [m^3]
        l = 0.27;   % [m] wing span
        rc = 0.27/4;   % raio do centro
        contraroting_dist = .1;     % [m] altura
    
        Cp = diag([2.5; 2.5; 9.99])*1e-2;    % coeficiente de arrasto de translacao
        Cr = diag([1.25; 1.25; 1.25/2])*1e-2;    % coeficiente de arrasto de rotacao
        
        maxAngAir = deg2rad(30); % [rad]
        maxAngWat = deg2rad(65); % [rad]
        satAngAir;      % saturacao de angulo de referencia
        satAngWat;      % saturacao de angulo de referencia
        
        satRot = saturation(deg2rad(179)*[-1; 1]); % satura rotacao pra evitar singularidade em R()
        
        mot = {}; % motores
        % direcionameto dos motores inferiores
        alfa = deg2rad([0; 0; 0; 0]);
        beta = deg2rad([0; 90; 0; 90]);
        
        % controladores
        pidZ;
        pidAtt;
        
        cor; % cor de plot
        waypoint_dist = 1;
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % environment parameters
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        env;    % variavel de ambiente
        rho_air = 1.293;    % [kg/m^3]   air density
        rho_wat = 1.0e3;    % [kg/m^3]   water density
        g = [0; 0; 9.78];   % gravidade [m/s^2]
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % time
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        dt = (1/100);   % 100Hz de frequencia interna
        t = 0;          % relogio
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    properties (SetAccess = private)
        x;      % vetor de estados
        ref;    % referencias de posicao e angulo
        u;      % vetor de entradas
        F = zeros(4,1), M = zeros(4,1); % forces and moments
        
        hyst;   % historico das variaveis
    end
