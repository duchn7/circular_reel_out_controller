Flight controller for rigid-wing groundgen airborne wind energy systems during circular reel out.

This depository is the supplementary material of paper 'Controlling rigid-wing airborne wind energy systems during circular flight without exact path following' by Duc H. Nguyen, Agusti Porta Ko, Tallak Tveide, Mark H. Lowenberg, and Espen Oland, published on Wind Energy Science in 2026, DOI: to be added.

The code was written in Julia but can be easily translated into other programming languages using a large language model.

The control law is represented by a system of 7 first-order ordinary differential equations. Due to the low gains used in the outer loop, the state i_x_P_error should be initialised at a non-zero value to facilitate quick convergence to the target cylinder and prevent unstable responses. For example, results in section 4.1 of the paper were all generated with initial i_x_P_error values that result in lambda_R equals to roughly 30 deg.
