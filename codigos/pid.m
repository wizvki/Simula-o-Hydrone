classdef pid < handle
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    properties (SetAccess = private, GetAccess = private)
        % Ganhos do Pid
		Kp = 1;
		Ti = inf;
		Td = 0;
		
		% referencia
		rn = 0.0
		
		% Erros ao longo do tempo discretizado
		ePn1  = 0.0 %Erro proporcional um tempo no passado
		eDfn1 = 0.0 %Erro derivativo um tempo no passado
		eDfn2 = 0.0 %Erro derivativo dois tempos no passado
		
		% saida de controle
		umin = -inf;
		umax = inf;
		un = 0.0;
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    methods
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % construtor
        function this = pid(Kp, Ti, Td, ulimits)
            this.Kp = Kp;
            this.Ti = Ti;
            this.Td = Td;
            
            this.reset();
            
            this.umin = ulimits(1);
            this.umax = ulimits(2);
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function reset(this)
            % erros guardados no objeto
            this.ePn1  = 0.0; 
            this.eDfn1 = 0.0;
            this.eDfn2 = 0.0;
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % atualiza estado do motor
        function u = getU(this, rn, yn, dt)
            
            % referência
            this.rn = rn;
            
            % parametros default do algoritmo
            BETA  = 1.0; %Age diretamente no ganho proporcional, uma variável de ajuste (?)
            ALPHA = 0.1; %Determina quantos % do tempo derivativo será o filtro da parte derivativa
            GAMMA = 0.0; %Age diretamente no ganho derivativo, uma variável de ajuste (?)

            % tempo de amostragem
            Ts = dt;

            % Erro proporcional com referencial de balanceamento
            ePn = (BETA*this.rn) - yn;

            % Cálculo do erro (utilizado apenas na parte integral)
            en = this.rn - yn;

            % Calcula o período do filtro passa-baixa para o ganho
            % derivativo
            Tf = ALPHA*this.Td;
            if (Tf ~= 0.0) && (Ts ~= -Tf)
                % Erro derivativo 
                eDn = (GAMMA*this.rn) - yn;
                % Filtro para o erro derivativo, afim de eliminar altas
                % frequências
                eDfn = (this.eDfn1/((Ts/Tf) + 1)) + (eDn*(Ts/Tf)/((Ts/Tf) + 1));
            else
                eDfn = 0.0;
            end
            % Resolução do compensador PID ótimo
            % (http://www.ece.ufrgs.br/~fetter/eng04037/pid.pdf)
            % delta de atuacao Proporcional
            delta_un = this.Kp*(ePn - this.ePn1);

            % termo integral
            if (this.Ti ~= 0.0)
                delta_un = delta_un + this.Kp*(Ts/this.Ti)*en;
            end

            % termo derivativo
            if (Ts ~= 0.0)
                delta_un = delta_un + this.Kp*(this.Td/Ts)*(eDfn - 2*this.eDfn1 + this.eDfn2);
            end

            % incrementa saida
            this.un = this.un + delta_un;

            % Limitador, utilizando a lógica anti-windup, limitada pela
            % alimentação do sistema simulado
            if this.un > this.umax
                this.un = this.umax;
            end
            if this.un < this.umin
                this.un = this.umin;
            end

            % atualização dos valores de cada iteração anterior
            this.ePn1 = ePn;
            this.eDfn2 = this.eDfn1;
            this.eDfn1 = eDfn;
            
            % retorna a nova saida			
            u = this.un;
        end
    end
end