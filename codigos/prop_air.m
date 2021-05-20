classdef prop_air < handle
%     Este codigo descreve o conjunto hélice-motor para o ar
    properties (SetAccess = private, GetAccess = private)
%         Parametros do sistema motor-helice
        w;              % velocidade do motor
        kg = 30;        % ganho do motor
        r = 0.250/2;    % raio da helice
        
%         funcoes de saturação de velocidade desejada conforme meio
        satW_air = saturation([0 4000]);
        satW_wat = saturation([0 4000]);
        
%         propriedades dos meios
        env;                % ambiente
        rho_air = 1.293;    % [kg/m^3]   massa específica do ar
        rho_wat = 1.0e3;    % [kg/m^3]   massa específica da água
    end

    methods
        function this = prop_air(env)
%             Construtor da classe
            this.env = env;
            this.w = 0;
        end
        
        function update(this, wdes, dt, env)     
%             Atualiza estado do conjunto motor-helice
            this.env = env;
            if isAir(this.env)
                wdes = this.satW_air.evaluate(wdes);
            else
                wdes = this.satW_wat.evaluate(wdes);
            end
            
%             eq dinamica do motor
            dw = this.kg*(wdes - this.getSpeed());
%             integracao da velocidade de rotacao
            this.w = this.getSpeed() + dw*dt;
        end

        function F = getForce(this)
%             Retorna força gerada pelo motor
            F = this.getKf()*this.getRho()*(this.getSpeed().^2);
        end

        function w = getSpeed(this)
%             Retorna velocidade de rotacao
            w = this.w;
            if isnan(w)
                w = 0;
            end
        end

        function kf = getKf(this)
%             Retorna coeficiente
%             fonte: maia 2017 naviator
            if isAir(this.env)
                kf = 2.45e-7;
            else
                kf = 2.55e-7*1e-3;
            end
        end

        function rho = getRho(this)
%             Retorna massa específica do ambiente
            if isAir(this.env)
                rho = this.rho_air;
            else
                rho = this.rho_wat;
            end
        end
     
        function draw(this, R, p, pah)
%             Desenha helice
            n = 20;                 % resolucao
            color = [0 0.2 1.0];    % cor da helice

%             Desenha helice
            if abs(this.getSpeed()) > 1
                alpha = .5; % rodando
            else
                alpha = .01; % parado
            end
            
            c = circle(this.r, n) + pah;
            [x, y, z] = feval(@(a)a{:}, num2cell(p));
            c = R*c + [x*ones(1,n); y*ones(1,n); z*ones(1,n)];
            %
            patch(c(1,:), c(2,:), c(3,:), color, 'FaceAlpha', alpha);
        end
    end
end