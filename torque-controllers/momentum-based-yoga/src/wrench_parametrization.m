function [f,F] = wrench_parametrization(delta_c, xi, delta_x, delta_y, delta_z)
    
    f_L = [(sqrt(delta_c))*(tanh(xi(1)))*(exp(xi(3)))/(1+(tanh(xi(2))^2))^0.5;
           (sqrt(delta_c))*(tanh(xi(2)))*(exp(xi(3)))/(1+(tanh(xi(1))^2))^0.5;
           exp(xi(3));
           delta_y*(tanh(xi(4)))*(exp(xi(3)));
           delta_x*(tanh(xi(5)))*(exp(xi(3)));
           delta_z*(tanh(xi(6)))*(exp(xi(3)))];

     F_L = [(sqrt(delta_c))*(exp(xi(3)))*(1-(tanh(xi(1)))^2)/(1+(tanh(xi(2)))^2)^0.5                             , (sqrt(delta_c))*(exp(xi(3)))*(tanh(xi(1)))*(tanh(xi(2)))*(-1+(tanh(xi(2)))^2)/(1+(tanh(xi(2)))^2)^1.5, (sqrt(delta_c))*(exp(xi(3)))*(tanh(xi(1)))/(1+(tanh(xi(2)))^2)^0.5, 0                                       , 0                                       , 0;
            (sqrt(delta_c))*(exp(xi(3)))*(tanh(xi(1)))*(tanh(xi(2)))*(-1+(tanh(xi(1)))^2)/(1+(tanh(xi(1)))^2)^1.5, (sqrt(delta_c))*(exp(xi(3)))*(1-(tanh(xi(2)))^2)/(1+(tanh(xi(1)))^2)^0.5                             , (sqrt(delta_c))*(exp(xi(3)))*(tanh(xi(2)))/(1+(tanh(xi(1)))^2)^0.5, 0                                       , 0                                       , 0;
            0                                                                                                    , 0                                                                                                    , exp(xi(3))                                                        , 0                                       , 0                                       , 0;
            0                                                                                                    , 0                                                                                                    , delta_y*(tanh(xi(4)))*(exp(xi(3)))                                , delta_y*(1-(tanh(xi(4)))^2)*(exp(xi(3))), 0                                       , 0;
            0                                                                                                    , 0                                                                                                    , delta_x*(tanh(xi(5)))*(exp(xi(3)))                                , 0                                       , delta_x*(1-(tanh(xi(5)))^2)*(exp(xi(3))), 0;
            0                                                                                                    , 0                                                                                                    , delta_z*(tanh(xi(6)))*(exp(xi(3)))                                , 0                                       , 0                                       , delta_z*(1-(tanh(xi(6)))^2)*(exp(xi(3)))];                 

     f_R = [(sqrt(delta_c))*(tanh(xi(7)))*(exp(xi(9)))/(1+(tanh(xi(8))^2))^0.5;
           (sqrt(delta_c))*(tanh(xi(8)))*(exp(xi(9)))/(1+(tanh(xi(7))^2))^0.5;
           exp(xi(9));
           delta_y*(tanh(xi(10)))*(exp(xi(9)));
           delta_x*(tanh(xi(11)))*(exp(xi(9)));
           delta_z*(tanh(xi(12)))*(exp(xi(9)))];

     F_R = [(sqrt(delta_c))*(exp(xi(9)))*(1-(tanh(xi(7)))^2)/(1+(tanh(xi(8)))^2)^0.5                             , (sqrt(delta_c))*(exp(xi(9)))*(tanh(xi(7)))*(tanh(xi(8)))*(-1+(tanh(xi(8)))^2)/(1+(tanh(xi(8)))^2)^1.5, (sqrt(delta_c))*(exp(xi(9)))*(tanh(xi(7)))/(1+(tanh(xi(8)))^2)^0.5, 0                                        , 0                                        , 0;
            (sqrt(delta_c))*(exp(xi(9)))*(tanh(xi(7)))*(tanh(xi(8)))*(-1+(tanh(xi(7)))^2)/(1+(tanh(xi(7)))^2)^1.5, (sqrt(delta_c))*(exp(xi(9)))*(1-(tanh(xi(8)))^2)/(1+(tanh(xi(7)))^2)^0.5                             , (sqrt(delta_c))*(exp(xi(9)))*(tanh(xi(8)))/(1+(tanh(xi(7)))^2)^0.5, 0                                        , 0                                        , 0;
            0                                                                                                    , 0                                                                                                    , exp(xi(9))                                                        , 0                                        , 0                                        , 0;
            0                                                                                                    , 0                                                                                                    , delta_y*(tanh(xi(10)))*(exp(xi(9)))                               , delta_y*(1-(tanh(xi(10)))^2)*(exp(xi(9))), 0                                        , 0;
            0                                                                                                    , 0                                                                                                    , delta_x*(tanh(xi(11)))*(exp(xi(9)))                               , 0                                        , delta_x*(1-(tanh(xi(11)))^2)*(exp(xi(9))), 0;
            0                                                                                                    , 0                                                                                                    , delta_z*(tanh(xi(12)))*(exp(xi(9)))                               , 0                                        , 0                                        , delta_z*(1-(tanh(xi(12)))^2)*(exp(xi(9)))];                 


     f = [f_L ; f_R];
     F = [F_L, F_R]; 
 end


