classdef Stage_cost_QR
    %STAGE_COST_QR A simple QR stage cost
    %   The stage cost exists out of two matrixes Q and R.
    %   - Q: Matrix penalty on the states.
    %   - R:  Matrix bounty on the inputs.
    %   The total stage cost can then be represented by: x^T Q x + u^T R u

    properties
        Q % Matrix penalty on the states.
        R % Matrix penalty on the inputs.
        Rd % Matrix penalty on the difference of inputs
        model % Model object of the system.
    end

    methods
        function obj = Stage_cost_QR(model,Q,R,Rd)
            obj.Q=Q;
            obj.R=R;
            obj.model=model;
            if (nargin > 3) % There is a delta u weight
                obj.Rd=Rd;
            else
                obj.Rd=zeros(size(R));
            end
        end
        function stage_cost=evaluate_cost(obj,state,input,iteration_index,...
                state_reference,input_reference,old_input)
            % Evaluates the stage cost at a certain state.
            %   - state = current state
            %   - input = Input applied to get the next state.
            %   - state_reference = The preferred state.
            %   - input_reference = The preferred inputs.
            %   - old_input = The last input, u0, used to calculate delta u.
            stage_cost=0;

            % Create input difference
            lo = length(old_input);
            input_difference = zeros(size(input));
            input_difference(1:lo) = input(1:lo) - old_input;
            input_difference((lo+1):end) = input((lo+1):end) - input(1:(end-lo));

            for i_col=1:obj.model.number_of_states
                for i_row=1:obj.model.number_of_states
                    stage_cost = stage_cost + (state(i_col)-state_reference(i_col))*...
                                obj.Q(i_col,i_row)*...
                                (state(i_row)-state_reference(i_row));
                end
            end

            for i_col=1:obj.model.number_of_inputs
                for i_row=1:obj.model.number_of_inputs
                    stage_cost = stage_cost + (input(i_col)-input_reference(i_col))*...
                                obj.R(i_col,i_row)*...
                                (input(i_row)-input_reference(i_row));
                end
            end

            for i_col=1:obj.model.number_of_inputs
                for i_row=1:obj.model.number_of_inputs
                    stage_cost = stage_cost + input_difference(i_col)*...
                                obj.Rd(i_col,i_row)*...
                                input_difference(i_row);
                end
            end
        end
    end

end

