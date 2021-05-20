methods (Access = public)
    % construtor
    function this = quadHibridoproposta(x, cor)
        
        % saturacoes de angulos
        this.satAngAir = saturation(this.maxAngAir*[-1; 1]);
        this.satAngWat = saturation(this.maxAngWat*[-1; 1]);
        
        % condicoes iniciais
        this.x = x;
        this.ref = this.x(1:6);
        
        % define o ambiente inicial
        this.setEnvironment();
        
        % inicializa os motores
        for m = 1:4
            this.mot{m} = prop_air_water(this.l, this.env);
            this.u(m) = this.mot{m}.getSpeed();
        end
        
        % historicos
        this.hyst.x = this.x(:);
        this.hyst.u = this.u(:);
        this.hyst.F = this.F(:);
        this.hyst.ref = this.ref(:);
        this.hyst.t = this.t;
        
        this.cor = cor;
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % PIDS (air) -- OK
        satz = 1000;
        satang = 3000;
        % z
        this.pidZ.air = pid(2.5e3, 1e0, 3e-1, satz*[-1, 1]);
        % roll and pitch
        for i = 1:2
            this.pidAtt.air{i} = pid(3e2, 1e2, 3e-1, satang*[-1, 1]);
        end
        % yaw
        this.pidAtt.air{3} = pid(1e2, 1e3, 1e-1, satang*[-1, 1]);
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % PIDS (water)
        satz = 1000;
        satang = 3000;
        % z
        this.pidZ.wat = pid(2.5e4, 1, 3e-1, satz*[-1, 1]);
        % pitch e yaw
        for i = 1:2
            this.pidAtt.wat{i} = pid(1e3, 1e3, 1e-2, satang*[-1, 1]);
        end
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % update do modelo
    function update(this, ref)
        
        % controle de posicao
        [dwF, dv, rref] = this.controllerPos(ref);
        
        % controle de atitude
        dw = this.controllerAtt(rref);
        
        % calcula a atuacao
        this.actuator(dwF, dv, dw);
        
        % dinamica
        this.dynamics();
        
        % historicos
        posref = ref(1:3);
        this.ref = [posref; rref];
        this.saveTraj();
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % salva trajetoria
    function saveTraj(this)
        this.hyst.x(:,end+1)    = this.x(:);
        this.hyst.u(:,end+1)    = this.u(:);
        this.hyst.F(:,end+1)    = this.F(:);
        this.hyst.ref(:,end+1)  = this.ref(:);
        this.hyst.t(end+1)      = this.t;
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % verifica se o robo chegou
    function c = reach(this)
        
        c = false;
        
        [x, y, z, psi] = feval(@(a)a{:}, num2cell(this.x([1 2 3 6])));
        [xr, yr, zr, psir] = feval(@(a)a{:}, num2cell(this.ref));
        
        % calcula distancia para waypoint corrent
        d = norm([xr yr zr]-[x y z], 2);
        if d <= this.waypoint_dist
            c = true;
        end
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    function resetController(this)
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % PIDS (air) -- OK
        % x e y
        for i = 1:2
            this.pidXY.air{i}.reset();
        end
        % z
        this.pidZ.air.reset();
        % roll and pitch
        for i = 1:2
            this.pidAtt.air{i}.reset();
        end
        % yaw
        this.pidAtt.air{3}.reset();
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % PIDS (water)
        % v           
        this.pidV.wat.reset();
        % z
        this.pidZ.wat.reset();
        % yaw
        this.pidAtt.wat{1}.reset();
        this.pidAtt.wat{2}.reset();
        
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % retorna o tempo de simulacao
    function t = time(this)
        t = this.t;
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % pega densidade do ambiente
    function rho = getRho(this)
        if isAir(this.env)
            rho = this.rho_air;
        else
            rho = this.rho_wat;
        end
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % desenha o quadrotor
    function draw(this)
        
        color = 'k'; % body color
        p = this.x(1:3);
        R = this.R(); % Monta matrizes de rotacao
        
        % shafts
        s{1} = this.l*[  1;  0; 0];
        s{2} = this.l*[  0; -1; 0];
        s{3} = this.l*[ -1;  0; 0];
        s{4} = this.l*[  0;  1; 0];

        % Aplica transformacoes: Body
        for i = 1:4
            b{i} = R*s{i} + p(:);
        end
        
        % draw shafts
        plot3([b{1}(1) b{3}(1)], [b{1}(2) b{3}(2)], [b{1}(3) b{3}(3)], color, 'linewidth', 2); 
        hold on;
        plot3([b{2}(1) b{4}(1)], [b{2}(2) b{4}(2)], [b{2}(3) b{4}(3)], color, 'linewidth', 2);

        % desenha propulsor
        for i = 1:4
            sp{i} = s{i} + [0; 0; (this.contraroting_dist/2)];
            sp{i+4} = s{i} - (this.contraroting_dist/2)*[   cos(this.alfa(i))*sin(this.beta(i)); 
                                                            sin(this.alfa(i))*sin(this.beta(i));  
                                                            cos(this.beta(i))];
            % defini o tipo do propulsor
            if mod(i, 2)
                type = [];
            else
                type = ['proposta'];
            end
            this.mot{i}.draw(R, p, sp{i}, sp{i+4}, type);
            %
            sp{i} = R*sp{i} + p(:);
            sp{i+4} = R*sp{i+4} + p(:);
            %
            plot3([sp{i}(1) b{i}(1)], [sp{i}(2) b{i}(2)], [sp{i}(3) b{i}(3)], ...
                color, 'linewidth', 2);
            plot3([b{i}(1) sp{i+4}(1)], [b{i}(2) sp{i+4}(2)], [b{i}(3) sp{i+4}(3)], ...
                color, 'linewidth', 2);
            
            %%number of propeller
            %text(sp{i}(1), sp{i}(2), sp{i}(3)+.1, num2str(i))
        end
        
        % draw body
        this.body();
        
        % plota trajetoria
        ns = 10;
        z = this.hyst.x(3,:);
        id1 = find(z >= 0);
        id2 = find(z < 0);
        % trajetoria aerea
        plot3(this.hyst.x(1,id1), this.hyst.x(2,id1), this.hyst.x(3,id1), ...
            '-', 'Color', .7*[1 0 0], 'linewidth', 2);
        % trajetÃ³ria aquatica
        plot3(this.hyst.x(1,id2), this.hyst.x(2,id2), this.hyst.x(3,id2), ...
            '-', 'Color', .7*[1 0 1], 'linewidth', 2);
        
        %%plota referencia
        %plot3(this.ref(1), this.ref(2), this.ref(3), 'rx');
        
        xlabel('x[m]')
        ylabel('y[m]')
        zlabel('z[m]')
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    function body(this)
        
        % set up unit sphere information
        [unitSphereX, unitSphereY, unitSphereZ] = sphere(8);

        % for each given sphere, shift the scaled unit sphere by the
        % location of the sphere and plot
        sphereX = this.x(1) + unitSphereX*this.rc;
        sphereY = this.x(2) + unitSphereY*this.rc;
        sphereZ = this.x(3) + 1*unitSphereZ*this.rc;
        h = surface(sphereX, sphereY, sphereZ);
        %rotate(h, [1 0 0], 25);

        % cor preta
        colormap(.5*[.8 .8 1]);
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % plota resultados
    function plot(this)
        
        figure(30)
        % resultados
        t = this.hyst.t;
        ref = this.hyst.ref;
        p = this.hyst.x(1:3,:);
        r = this.hyst.x(4:6,:);
        v = this.hyst.x(7:9,:);
        q = this.hyst.x(10:12,:);
        w = this.hyst.u;
        F = this.hyst.F;

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %figure('units','normalized','outerposition',[0 0 1 1]);
        labels = {'x', 'y', 'z'};
        for i = 1:3
            subplot(3,2,2*i-1)
            plot(t, p(i,:), 'Color', this.cor, 'linewidth', 1); hold on;
            plot(t, ref(i,:), '--', 'Color', this.cor, 'linewidth', 1); hold on;
            ylabel(['$$' labels{i} '$$ [m.]'], 'Interpreter','latex')
            xlim([t(1) t(end)])
            box off;
            xlabel('time [sec.]')
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%
        labels = {'\phi', '\theta', '\psi'};
        sp = [2 1 3];
        for i = 1:3
            subplot(3,2,2*sp(i))
            plot(t, rad2deg(r(i,:)), 'Color', this.cor, 'linewidth', 1); hold on;
            plot(t, rad2deg(ref(3+i,:)), '--', 'Color', this.cor, 'linewidth', 1); hold on;
            ylabel(['$$' labels{i} '$$ [deg.]'], 'Interpreter','latex')
            xlim([t(1) t(end)])
            box off;
            xlabel('time [sec.]')
        end

        % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % %%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % figure('units','normalized','outerposition',[0 0 1 1]);
        % labels = {'v_x', 'v_y', 'v_z'};
        % for i = 1:3
        %     subplot(3,2,2*i-1)
        %     plot(t, v(i,:), 'Color', this.cor, 'linewidth', 1); hold on;
        %     ylabel(['$$' labels{i} '$$ [m./sec.]'], 'Interpreter','latex')
        %     xlim([t(1) t(end)])
        %     box off;
        %     xlabel('time [sec.]')
        % end
        % %%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % labels = {'p', 'q', 'r'};
        % for i = 1:3
        %     subplot(3,2,2*i)
        %     plot(t, rad2deg(q(i,:)), 'Color', this.cor, 'linewidth', 1); hold on;
        %     ylabel(['$$' labels{i} '$$ [deg./sec.]'], 'Interpreter','latex')
        %     xlim([t(1) t(end)])
        %     box off;
        %     xlabel('time [sec.]')
        % end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        figure(32)
        for i = 1:4
            subplot(4,1,i)
            plot(t, w(i,:), 'Color', this.cor, 'linewidth', 1); 
            hold on;
            ylabel('$$\vec{\Omega}$$ [rpm]', 'Interpreter','latex')
            xlim([t(1) t(end)])
            box off;
            xlabel('time [sec.]')
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%             figure(33)
%             for i = 1:4
%                 subplot(4,1,i)
%                 plot(t, F(i,:), 'Color', this.cor, 'linewidth', 1); 
%                 hold on;
%                 ylabel('$$\vec{F}$$ [N]', 'Interpreter','latex')
%                 xlim([t(1) t(end)])
%                 box off;
%                 xlabel('time [sec.]')
%             end
    end
end
end