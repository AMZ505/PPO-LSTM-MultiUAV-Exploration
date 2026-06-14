classdef GridWorldTSP60 < matlab.System

    
    % Public, tunable properties
    properties(Access = private)
        % Initial position of robots
        InitialStates= [2 5; 11 5; 3 5; 6 5; 3 3; 3 4; 6 4; 10 2]
    end           

    % Public, non-tunable properties
    properties(Nontunable)
        % Obstacle matrix
        Obstacles double = -10
        % Max step count
        MaxStepCount (1,1) double = 500
    end

    properties(DiscreteState)
        % Discretized XY space with cells containing 0.5 or 1.0
        % 0:   unexplored
        % 0.25: explored by robot A
        % 0.50: explored by robot B
        % 0.75: explored by robot C
%         1.25 explored by robot d
        % 10.0: obstacle
        Grid
        % States of robot A,B: [rowA colA; rowB colB]
        States

         % Previous States of robot A,B: [rowA colA; rowB colB]
        PStates
        % Step count
        StepCount
        % Individual cell exploration count
        NumExploredCells
        State
    end

    % Pre-computed constants
    properties(Access = private)
        % Handle to figure
        Figure
        % Grid size [numrows numcols]
        Size (1,2) double = [12 12]
    end

    methods
        % Constructor
        function this = GridWorldTSP60(varargin)
            % Support name-value pair arguments when constructing object
            setProperties(this,nargin,varargin{:})
        end
    end

    methods(Access = protected)
        %% Common functions
        function setupImpl(obj) %#ok<MANU>
            % Perform one-time calculations, such as computing constants
        end

         function [observations,rewards,isdone] = stepImpl(obj,actions)
            global k1;
            global k2;
            global k3;
            global k4;
            global S;
            global H;
            global TSP;
            global TSP1;
            global TSP2;
            global TSP3;
            global TSP4;
            global TSP5;
            global TSP6;
            global TSP7;
            global TSP8;
            global Explored
            
%             global BAA;
            % Implement algorithm. Calculate y as a function of input u and
            % discrete states.
            
            numRobots =8;
%             dbup 4; ff=length(actionSpace);
%             dbup;
            
            % Rewards are:
            % Agent moves to unexplored cell: +20
            %%%%%%%%%%%%Agent moves to explored cell: -1
            % Agent moves to explored cell: 0
            % Agent tries to move out of grid: -10
            % Agent collides with another agent: -10
            % Agent collides with obstacle: -10
            % Movement penalty: -1
            % Lazy penalty: -2
            % On full coverage: +4000 * coverage contribution
            
            % move robots to their next state
            next_states = zeros(numRobots,2);
            rewards = zeros(numRobots,1);
            isdone = 0;
            next_states = obj.States;
ff=[1,1; 2,1;3,1; 4,1;5,1;6,1;7,1;8,1;9,1;10,1;11,1;12,1;1,2; 2,2;3,2; 4,2;5,2;6,2;7,2;8,2;9,2;10,2;11,2;12,2;1,3; 2,3;3,3; 4,3;5,3;6,3;7,3;8,3;9,3;10,3;11,3;12,3;1,4; 2,4;3,4; 4,4;5,4;6,4;7,4;8,4;9,4;10,4;11,4;12,4;1,5; 2,5;3,5; 4,5;5,5;6,5;7,5;8,5;9,5;10,5;11,5;12,5;1,6; 2,6;3,6; 4,6;5,6;6,6;7,6;8,6;9,6;10,6;11,6;12,6;1,7; 2,7;3,7; 4,7;5,7;6,7;7,7;8,7;9,7;10,7;11,7;12,7;1,8; 2,8;3,8; 4,8;5,8;6,8;7,8;8,8;9,8;10,8;11,8;12,8;1,9; 2,9;3,9; 4,9;5,9;6,9;7,9;8,9;9,9;10,9;11,9;12,9;1,10; 2,10;3,10; 4,10;5,10;6,10;7,10;8,10;9,10;10,10;11,10;12,10;1,11; 2,11;3,11; 4,11;5,11;6,11;7,11;8,11;9,11;10,11;11,11;12,11;1,12; 2,12;3,12; 4,12;5,12;6,12;7,12;8,12;9,12;10,12;11,12;12,12];



% ff= [1,1; 2,1;3,1; 4,1;5,1;6,1;7,1;8,1;9,1;10,1;11,1;12,1;...
% 1,2; 2,2;3,2; 4,2;5,2;6,2;7,2;8,2;9,2;10,2;11,2;12,2;1,3; 2,3;3,3; 4,3;5,3;6,3;7,3;8,3;9,3;10,3;11,3;12,3;...
% 1,4; 2,4;3,4; 4,4;5,4;6,4;7,4;8,4;9,4;10,4;11,4;12,4;1,5; 2,5;3,5; 4,5;5,5;6,5;7,5;8,5;9,5;10,5;11,5;12,5;...
% 1,6; 2,6;3,6; 4,6;5,6;6,6;7,6;8,6;9,6;10,6;11,6;12,6;1,7; 2,7;3,7; 4,7;5,7;6,7;7,7;8,7;9,7;10,7;11,7;12,7;...
% 1,8; 2,8;3,8; 4,8;5,8;6,8;7,8;8,8;9,8;10,8;11,8;12,8;1,9; 2,9;3,9; 4,9;5,9;6,9;7,9;8,9;9,9;10,9;11,9;12,9;...
% 1,10; 2,10;3,10; 4,10;5,10;6,10;7,10;8,10;9,10;10,10;11,10;12,10;...
% 1,11; 2,11;3,11; 4,11;5,11;6,11;7,11;8,11;9,11;10,11;11,11;12,11;...
% 1,12; 2,12;3,12; 4,12;5,12;6,12;7,12;8,12;9,12;10,12;11,12;12,12];

% ff= [1,1; 2,1;3,1; 4,1;5,1;6,1;7,1;8,1;9,1;10,1;11,1;12,1;1,2; 2,2;3,2; 4,2;5,2;6,2;7,2;8,2;9,2;10,2;11,2;12,2;1,3; 2,3;3,3; 4,3;5,3;6,3;7,3;8,3;9,3;10,3;11,3;12,3;1,4; 2,4;3,4; 4,4;5,4;6,4;7,4;8,4;9,4;10,4;11,4;12,4;1,5; 2,5;3,5; 4,5;5,5;6,5;7,5;8,5;9,5;10,5;11,5;12,5;1,6; 2,6;3,6; 4,6;5,6;6,6;7,6;8,6;9,6;10,6;11,6;12,6;1,7; 2,7;3,7; 4,7;5,7;6,7;7,7;8,7;9,7;10,7;11,7;12,7;1,8; 2,8;3,8; 4,8;5,8;6,8;7,8;8,8;9,8;10,8;11,8;12,8;1,9; 2,9;3,9; 4,9;5,9;6,9;7,9;8,9;9,9;10,9;11,9;12,9;1,10; 2,10;3,10; 4,10;5,10;6,10;7,10;8,10;9,10;10,10;11,10;12,10;1,11; 2,11;3,11; 4,11;5,11;6,11;7,11;8,11;9,11;10,11;11,11;12,11;1,12; 2,12;3,12; 4,12;5,12;6,12;7,12;8,12;9,12;10,12;11,12;12,12];
            for idx = 1:numRobots


                
                state = obj.States(idx,:);
               
                action = actions(idx);
                if action~=0
                    next_states(idx,:)= ff(action,:);    
                end
%                 rewards(idx)=rewards(idx)-pdist2( next_states(idx,:),state);


                 %Check the best next 
                 TSPTemp=TSP;
                 if action~=0
                 TSPTemp(action,:)=[5000000000000,5000000000000];
%                  [~,i]=min(pdist2(TSP(action,:),TSPTemp),[],1);


                 % Check the if it is unexplored

                  if ~isempty(find(Explored==action,1))
%                      rewards(idx)=dist();
%                         rewards(idx)=rewards(idx)-10;
                  end



                  % PENALTY FOR LARGE NEXT DIST
%                       FIND THE DIST
%                       PENALIZED REWARD TO FIT



%                  else

                     %If was unexplored, save it
                     SAME=0;
                     if action~=0
                     if(obj.StepCount==1)
                            GK=0;
                        else
                            GK=obj.StepCount-1;
                     end



                        if isempty(find(Explored==action,1)) &&  action~=0 %&&  (length(nonzeros(Explored))<140)
%                             if idx+(3*GK)<=144
                                HG=find(Explored==0, 1, 'first');
                                Explored(1, HG)=action; 
%                             end
                        else
%                             rewards(idx)=rewards(idx)-10;
                            SAME=1;
                        end
                     end


%                      if action~=0
%                         Explored(1, idx+obj.StepCount-1)=action;
%                      end
%                         rewards(idx)=rewards(idx)+1;

                     if idx==1 && action~=0  
                          if(SAME==0)
                         if  1+obj.StepCount>144
                             hgjgf=0;
                         end
                          HG1=find(TSP1==0, 1, 'first');
                          TSP1(1, HG1)=action;
                          obj.Grid(ff(action,:)) = 0.25;
                         
                           obj.NumExploredCells = obj.NumExploredCells + [1 0 0 0 0 0 0 0];
                          end
                          

                     elseif idx==2 && action~=0
                                                  if  1+obj.StepCount>144
                             hgjgf=0;
                         end
                         if(SAME==0)
                          HG2=find(TSP2==0, 1, 'first');
                          TSP2(1, HG2)=action;
                          obj.Grid(ff(action,:)) = 0.5;
                           
                            obj.NumExploredCells = obj.NumExploredCells + [0 1 0 0 0 0 0 0];
                           end

                      elseif idx==3 && action~=0 
                                                   if  1+obj.StepCount>144
                             hgjgf=0;
                         end
                          if(SAME==0)
                          HG3=find(TSP3==0, 1, 'first');
                          TSP3(1, HG3)=action;
                          obj.Grid(ff(action,:)) = 0.75;
                           
                            obj.NumExploredCells = obj.NumExploredCells + [0 0 1 0 0 0 0 0];
                          end

                        elseif idx==4 && action~=0 
                         if  1+obj.StepCount>144
                             hgjgf=0;
                         end
                          if(SAME==0)
                          HG4=find(TSP4==0, 1, 'first');
                          TSP4(1, HG4)=action;
                          obj.Grid(ff(action,:)) = 1.25;
                           
                            obj.NumExploredCells = obj.NumExploredCells + [0 0  0 1 0 0 0 0];
                          end

                       elseif idx==5 && action~=0 
                         if  1+obj.StepCount>144
                             hgjgf=0;
                         end
                          if(SAME==0)
                          HG5=find(TSP5==0, 1, 'first');
                          TSP5(1, HG5)=action;
                          obj.Grid(ff(action,:)) = 1.5;
                           
                            obj.NumExploredCells = obj.NumExploredCells + [0 0  0  0 1 0 0 0];
                          end

                        elseif idx==6 && action~=0 
                         if  1+obj.StepCount>144
                             hgjgf=0;
                         end
                          if(SAME==0)
                          HG6=find(TSP6==0, 1, 'first');
                          TSP6(1, HG6)=action;
                          obj.Grid(ff(action,:)) = 1.75;
                           
                            obj.NumExploredCells = obj.NumExploredCells + [0 0  0  0 0  1 0 0];
                          end
                          
                                                  elseif idx==7 && action~=0 
                         if  1+obj.StepCount>144
                             hgjgf=0;
                         end
                          if(SAME==0)
                          HG7=find(TSP7==0, 1, 'first');
                          TSP7(1, HG7)=action;
                          obj.Grid(ff(action,:)) = 2;
                           
                            obj.NumExploredCells = obj.NumExploredCells + [0 0  0  0 0 0 1 0];
                          end

                                                                            elseif idx==8 && action~=0 
                         if  1+obj.StepCount>144
                             hgjgf=0;
                         end
                          if(SAME==0)
                          HG8=find(TSP8==0, 1, 'first');
                          TSP8(1, HG8)=action;
                          obj.Grid(ff(action,:)) = 2;
                           
                            obj.NumExploredCells = obj.NumExploredCells + [0 0  0  0 0 0 0 1];
                           end

                     end
                     obj.States = obj.InitialStates;

%                  end
                 end

                 % Check the energy
            end



             totalExploredCells = sum(Explored>0);
             totalCells = obj.Size(1) * obj.Size(2) ;%- sum(obj.Grid==10,'all');
              if totalExploredCells >= totalCells
                 isdone = 1;
                 Explored=zeros(1,144);
              end
                 
                 for idx=1:numRobots
                     if idx==1
                         ctsp=[];
                         ctsp=find(TSP1>0);
                     elseif idx==2
                         ctsp=[];
                         ctsp=find(TSP2>0);
                      elseif idx==3
                         ctsp=[];
                         ctsp=find(TSP3>0);
                      elseif idx==4
                         ctsp=[];
                         ctsp=find(TSP4>0);
                      elseif idx==5
                         ctsp=[];
                         ctsp=find(TSP5>0);
                      elseif idx==6
                         ctsp=[];
                         ctsp=find(TSP6>0);
                      elseif idx==7
                         ctsp=[];
                         ctsp=find(TSP7>0);
                      elseif idx==8
                         ctsp=[];
                         ctsp=find(TSP8>0);
                     end
                     if ~isempty(ctsp) && length(ctsp)>1
map = [ ...
    1, 1, 1, 1, 1,1, 1, 1, 1, 1,1,1  ; ...
    1, 1, 1, 1, 1,1, 1, 1, 1, 1,1,1  ; ...
    1, 1, 1, 1, 1,1, 1, 1, 1, 1,1,0  ; ...
    1, 0, 1, 1, 1,1, 1, 1, 1, 1,1,1  ; ...
    1, 0, 1, 1, 1,1, 1, 1, 1, 1,0,1  ; ...
    1, 0, 1, 1, 1,1, 1, 1, 1, 1,0,0  ; ...
    1, 0, 1, 1, 1,1, 1, 1, 1, 1,0,0  ; ...
    1, 0, 1, 1, 1,1, 1, 1, 1, 1,0,1  ; ...
    1, 0, 1, 1, 1,1, 1, 1, 1, 1,1,1  ; ...
    1, 1, 1, 1, 1,1, 1, 1, 1, 1,1,1  ; ...
    1, 1, 1, 1, 1,1, 1, 1, 1, 1,1,1  ; ...
    1, 1, 1, 1, 1,1, 1, 1, 1, 1,1,1  ; ...
   
    ];

costs = ones(size(map));
% [final, fScore] = a_star(logical(map), costs, TSP1(i-1),TSP1(i));








                        x = TSP(ctsp(end-1:end),1)   ;                          % Create Data
                        y = TSP(ctsp(end-1:end),2) ;                              % Create Data
%                     d = hypot(diff(x), diff(y))    ;                       % Distance Of Each Segment
                        [final, fScore] = a_star(logical(map), costs, ctsp(end-1), ctsp(end));
                        HH=H;
                        H=H+min(min(fScore));
                        if H==inf
                            H=HH;
                        end
                        d_tot=H;
                        rewards(idx)=rewards(idx)-(d_tot*.1);

                     end
                 end

%                  for idx=1:numRobots
%                         rewards(idx)  = rewards(idx)+ (10000 * (1)*(obj.NumExploredCells(idx)'/totalCells))  +(20000* (obj.MaxStepCount/obj.StepCount)*(totalExploredCells == totalCells));
% %                         + (20000 * (obj.MaxStepCount/obj.StepCount)*(obj.NumExploredCells(idx)'/totalCells))
%                  end
% %                 rewards = rewards+round((obj.MaxStepCount/obj.StepCount)*totalExploredCells);
              
            
            % Observation for each agent is a 12x12 2-channel image. The
            % channels are:
            % 1. Obstacle channel - cells with obstacles are 1, rest 0
            % 2. Self channel - cell with the agent's state is 1, rest 0
            % 3. Friend channel - cell with other agent's state is 1, rest 0
            % 4. Coverage channel - cells that are unexplored are 1, rest 0
            observations = zeros(obj.Size(1),obj.Size(2),2,numRobots);
            for idx = 1:numRobots
                obstacleChannel = 1.0 * (obj.Grid == 10.0);
                
                selfIdx = idx;
                selfRow = next_states(selfIdx,1);
                selfCol = next_states(selfIdx,2);
                selfChannel = zeros(obj.Size);
                selfChannel(selfRow,selfCol) = 1.0;
                
                friendIdxs = idx ~= (1:numRobots);
                friendRows = next_states(friendIdxs,1);
                friendCols = next_states(friendIdxs,2);
                friendChannel = zeros(obj.Size);
                friendChannel(friendRows(1),friendCols(1)) = 1.0;
                friendChannel(friendRows(2),friendCols(2)) = 1.0;
                
                coverageChannel = 1.0 * (obj.Grid == 0);
                
                observations(:,:,1,idx) = obstacleChannel;
                observations(:,:,2,idx) = selfChannel;
                observations(:,:,3,idx) = friendChannel;
                observations(:,:,4,idx) = coverageChannel;
            end
            
            % Scale down rewards
            rewards = rewards./20;
            
            % DEBUG
            if all(next_states(1,:)==next_states(2,:)) || ...
                    all(next_states(2,:)==next_states(3,:)) || ...
                    all(next_states(1,:)==next_states(3,:))
                fprintf('Assertion: Invalid state.\n');
            end
            
            % Update states
            obj.States = next_states;
            obj.StepCount = obj.StepCount + 1;
            
            % plot the environment
            if obj.StepCount <= obj.MaxStepCount
                % plot(obj);
            end


%             M.Time=obj.StepCount;
%             M.Collision=H-25;
%             M.Performance=totalExploredCells/totalCells ;
%             M.Energy=sum(sum(energy));
%             M.SameExplored=(k1+k2+k3)-(3*25);
%             M.NumReturns=R;
%             M.Actions=S;
%             M.NotExplored=totalCells-totalExploredCells;
%             M.InitialStages=obj.InitialStates;
%             M.Obs=obj.Obstacles;
            save('f4');
%             save('f4');
            
        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties

            
            % set unexplored cells
            obj.Grid = zeros(obj.Size);
            obj.InitialStates= [2 5; 11 5; 3 4; 3 3; 3 5; 3 9;6 4; 10 2];
            
            % set obstacle cells
            if ~isequal(obj.Obstacles,-10)
                for idx = 1:size(obj.Obstacles,1)
                    r = obj.Obstacles(idx,1);
                    c = obj.Obstacles(idx,2);
                    obj.Grid(r,c) = 10.0;
                end
            end
            
            % set step count
            obj.StepCount = 0;
            
            % explored cells
            obj.NumExploredCells = ones(1,8);
            
            % set robot cells
            sA = obj.InitialStates(1,:);
            sB = obj.InitialStates(2,:);
            sC = obj.InitialStates(3,:);
            sD = obj.InitialStates(4,:);
            sE = obj.InitialStates(5,:);
            sF = obj.InitialStates(6,:);
            sG = obj.InitialStates(7,:);
            sH = obj.InitialStates(8,:);
            %% 
% 
            obj.Grid(sA(1),sA(2)) = 0.25;
            obj.Grid(sB(1),sB(2)) = 0.50;
            obj.Grid(sC(1),sC(2)) = 0.75;
            obj.Grid(sD(1),sD(2)) = 1.25;
            obj.Grid(sE(1),sE(2)) = 1.5;
            obj.Grid(sF(1),sF(2)) = 1.75;
            obj.Grid(sG(1),sG(2)) = 2;
            obj.Grid(sH(1),sH(2)) = 2.25;
            obj.States = obj.InitialStates;
        end

        %% Backup/restore functions
        function s = saveObjectImpl(obj)
            % Set properties in structure s to values in object obj

            % Set public properties and states
            s = saveObjectImpl@matlab.System(obj);

            % Set private and protected properties
            %s.myproperty = obj.myproperty;
        end

        function loadObjectImpl(obj,s,wasLocked)
            % Set properties in object obj to values in structure s

            % Set private and protected properties
            % obj.myproperty = s.myproperty; 

            % Set public properties and states
            loadObjectImpl@matlab.System(obj,s,wasLocked);
        end

        %% Simulink functions
        function ds = getDiscreteStateImpl(obj)
            % Return structure of properties with DiscreteState attribute
            ds.Grid = obj.Grid;
            ds.States = obj.States;
            ds.StepCount = obj.StepCount;
            ds.NumExploredCells = obj.NumExploredCells;
        end

        function flag = isInputSizeMutableImpl(obj,index) %#ok<INUSD>
            % Return false if input size cannot change
            % between calls to the System object
            flag = false;
        end

        function [out1, out2, out3] = getOutputSizeImpl(obj) %#ok<MANU>
            % Return size for each output port
            out1 = [12 12 4 8];  % observation
            out2 = [8 1];   % reward
            out3 = [1 1];   % isdone
        end

        function [out1,out2,out3] = getOutputDataTypeImpl(obj) %#ok<MANU>
            % Return data type for each output port
            out1 = "double";
            out2 = "double";
            out3 = "double";
            
        end

        function [out1,out2,out3] = isOutputComplexImpl(obj) %#ok<MANU>
            % Return true for each output port with complex data
            out1 = false;
            out2 = false;
            out3 = false;
            out4 = false;
            out5 = false;
            out6 = false;
            out7= false;
            out8= false;

        end

        function [out1,out2,out3] = isOutputFixedSizeImpl(obj) %#ok<MANU>
            % Return true for each output port with fixed size
            out1 = true;
            out2 = true;
            out3 = true;
            out4= true;
            out5= true;
            out6= true;
            out7= true;
            out8= true;

     
        end

        function [sz,dt,cp] = getDiscreteStateSpecificationImpl(obj,name)
            % Return size, data type, and complexity of discrete-state
            % specified in name
            if strcmpi(name,'Grid')
                sz = obj.Size;
                dt = "double";
                cp = false;
            elseif strcmpi(name,'States')
                sz = [8 2];
                dt = "double";
                cp = false;
            elseif strcmpi(name,'StepCount')
                sz = [1 1];
                dt = "double";
                cp = false;
            elseif strcmpi(name,'NumExploredCells')
                sz = [1 8];
                dt = "double";
                cp = false;
            else
                error(['Error: Incorrect State Name: ', name.']);
            end
        end

        function icon = getIconImpl(obj) %#ok<MANU>
            % Define icon for System block
            icon = mfilename("class"); % Use class name
        end
    end

    methods(Static, Access = protected)
        %% Simulink customization functions
        function header = getHeaderImpl
            % Define header panel for System block dialog
            header = matlab.system.display.Header(mfilename("class"));
        end

        function group = getPropertyGroupsImpl
            % Define property section(s) for System block dialog
            group = matlab.system.display.Section(mfilename("class"));
        end

    end
    
    methods(Access=private)
        function collision = checkCollision(obj,new_state,other_states)
            % Create array of occupied states
            if isequal(obj.Obstacles,-10)
                occupiedStates = other_states;
            else
                occupiedStates = [other_states; obj.Obstacles];
            end
            % check collision
            collision = any(all(new_state==occupiedStates,2));
        end
        
        function plot(obj)
            persistent ax cells robots

            if isempty(ax) || ~isvalid(ax)
                % build figure
                % f = figure;
                % f.Position = [195 120 400 300];
                %f.Visible = 'on';  % force external figure
                % ax = gca(f);
                % hold(ax,'on');
                if ~isequal(obj.Obstacles,-10)
                    cmap = [255 255 255; ...    % white  (unexplored)
                            255 140 105; ...    % light red (explored by A)
                            152 251 152; ...    % light green (explored by B)
                            176 226 255; ...    % light blue (explored by C)
                            0 0 0]./255;        % black (obstacles)
                else
                    cmap = [255 255 255;...     % white  (unexplored)
                            255 140 105; ...    % light red (explored by A)
                            152 251 152; ...    % light green (explored by B)
                            176 226 255; ...    % light blue (explored by C)
                            ]./255;        % black (obstacles)
                end
                colormap(ax,cmap);
                
                % plot cells
                cdata = obj.Grid;
                cells = imagesc(ax,[0.5 obj.Size(1)-0.5],[0.5 obj.Size(2)-0.5],cdata);
                
                % plot grid lines
                x = 0:1:obj.Size(2);
                % y = 0:1:obj.Size(1);
                % [X,Y] = meshgrid(x,y);
                % plot(ax,X,Y,'Color',[0.94 0.94 0.94]);
                % plot(ax,Y,X,'Color',[0.94 0.94 0.94]);

                % plot robots
                sA = obj.InitialStates(1,:);
                sB = obj.InitialStates(2,:);
                sC = obj.InitialStates(3,:);
                sD = obj.InitialStates(4,:);
                sE = obj.InitialStates(5,:);
                sF = obj.InitialStates(6,:);
                sG = obj.InitialStates(7,:);
                sH = obj.InitialStates(8,:);
                robots    = gobjects(1,2);
                % robots(1) = rectangle(ax,'Position',[sA(2)-1 sA(1)-1 1 1],'FaceColor','r','Curvature',1);
                % robots(2) = rectangle(ax,'Position',[sB(2)-1 sB(1)-1 1 1],'FaceColor','g','Curvature',1);
                % robots(3) = rectangle(ax,'Position',[sC(2)-1 sC(1)-1 1 1],'FaceColor','b','Curvature',1);

                % time text
                totalExploredCells = sum(obj.NumExploredCells);
                totalCells = obj.Size(1) * obj.Size(2) - sum(obj.Grid==1,'all');
                coverage = totalExploredCells / totalCells * 100;
                % title(ax,sprintf('Steps = %d, Coverage = %.1f%%',obj.StepCount,coverage));

                ax.XTick = 0:1:obj.Size(1);
                ax.YTick = 0:1:obj.Size(2);
                ax.XTickLabel = {};
                ax.YTickLabel = {};
                % axis(ax,'equal');
                ax.XLim = [0 obj.Size(1)];
                ax.YLim = [0 obj.Size(2)];
                ax.Box = 'on';
            end
            
            % update color map
            cmap = [255 255 255; ...    % white  (unexplored)
                    255 140 105; ...    % light red (explored by A)
                    152 251 152; ...    % light green (explored by B)
                    176 226 255; ...
                    0 0 0]./255;        % black (obstacles)
            if all(obj.Grid~=0,'all')
                cmap = cmap(2:end,:);   % no unexplored cells
            end
            if isequal(obj.Obstacles,-10)
                cmap = cmap(1:end-1,:); % no obstacles
            end
            colormap(ax,cmap);
            
            % update cell colors
            cdata = obj.Grid;
            cells.CData = cdata;
            
            % update robot positions
            for idx = 1:5
                s = obj.States(idx,:);
                robots(idx).Position = [s(2)-1 s(1)-1 1 1];
            end
            
            % update info text
            totalExploredCells = sum(obj.NumExploredCells);
            totalCells = obj.Size(1) * obj.Size(2) - sum(obj.Grid==1,'all');
            coverage = totalExploredCells / totalCells * 100;
            % ax.Title.String = sprintf('Steps = %d, Coverage = %.1f%%',obj.StepCount,coverage);
            % 
            % ax.XLim = [0 obj.Size(1)];
            % ax.YLim = [0 obj.Size(2)];
            % grid(ax,'on');
            
            % drawnow();
        end
    end
end


