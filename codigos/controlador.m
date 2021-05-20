        % controlador de posicao
        function [dwF, dv, angref] = controllerPos(this, ref)
            
            % referencias de posicao
            [xref, yref, zref, psiref] = feval(@(a)a{:}, num2cell(ref));
            
            % medicoes
            [x, y, z, psi, vx, vy] = feval(@(a)a{:}, num2cell(this.x([1 2 3 6 7 8])));
            
            % calcula os sinais de referencia angular (***so roll e pitch***)
            angref = zeros(3,1);
            
            % calcula o angulo de direcao do controle
            dx = xref - x; %erro em x
            dy = yref - y; %erro em y
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % controle no ar
            if isAir(this.env)
                
                % ganhos de posicao
                kp =  0.05;
                kd =  0.11;
                
                % acao de controle
                R = rotz(psi);
                R = R(1:2,1:2);
                dp = R*[dy; dx];
                dv = -R*[vy; vx];
                angref = kp*[-1; 1].*dp + kd*[-1; 1].*dv; %Controle P, não utiliza a função pid.m
                angref = this.satAngAir.evaluate(angref);
                
                % referencia de yaw
                angref(3) = psiref;
                %
                dv = 0;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % controle na agua
            else
                % erro de posicao
                d = norm([dx, dy], 2);

                % comando de velocidade
                if d > this.waypoint_dist
                    
                    % erro angular: sempre entre [0...2pi]
                    th = atan2(dy,dx);
                    th = mod(th, 2*pi);
                    
                    % alpha entre [-pi..pi]
                    alpha = th - psi;
                    while alpha > pi
                        alpha = alpha - 2*pi;
                    end
                    while alpha < -pi
                        alpha = alpha + 2*pi;
                    end
                    
                    kv = -300;
                    ka = 1e4;
                    
                    %if abs(rad2deg(alpha)) > 35
                    %    dv = 0;
                    %else 
                        dv = kv*d + ka*abs(alpha)^2;
                    %end
                    % satura
                    dv = max(dv, -4000);
                    dv = min(dv, 0);
                else
                    dv = 0;
                    th = psi;
                end
                
                % referencia aponta para waypoint
                angref(3) = th;
            end
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % acao de controle de altitude
            if isAir(this.env)
                dwF = this.pidZ.air.getU(zref, z, this.dt); %utiliza a função pid.m
            else
                dwF = this.pidZ.wat.getU(zref, z, this.dt); %utiliza a função pid.m
            end
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % controlador de atitude
        function dw = controllerAtt(this, rref)
            
            r = this.x(4:6); % angulos medidos
            dw = zeros(3,1); % vetor de atuacao
            
            % calcula a atuação do controle a partir da função pid.m, os
            % ganhos são característicos para cada meio, descritos no
            % construtor da classe.
            if isAir(this.env)
                for i = 1:3
                    dw(i) = this.pidAtt.air{i}.getU(rref(i), r(i), this.dt);
                end
            else
                % alpha normalizado entre [-pi..pi]
                alpha = rref(3) - r(3);
                while alpha > pi
                    alpha = alpha - 2*pi;
                end
                while alpha < -pi
                    alpha = alpha + 2*pi;
                end
                dw(2) = this.pidAtt.wat{1}.getU(rref(2), r(2), this.dt); %utiliza a função pid.m
                dw(3) = this.pidAtt.wat{2}.getU(0, -alpha, this.dt);     %utiliza a função pid.m
            end
        end