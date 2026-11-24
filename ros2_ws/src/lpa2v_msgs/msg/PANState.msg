# Full output of a Paraconsistent Analysis Node (PAN) - PAL2v
std_msgs/Header header
float64 mu          # favorable evidence degree [0,1]
float64 lam         # unfavorable evidence degree [0,1]
float64 dc          # certainty degree [-1,1]
float64 dct         # contradiction degree [-1,1]
float64 dcr         # real certainty degree [-1,1]
float64 mu_er       # resulting real evidence degree [0,1]
float64 phi         # certainty interval [0,1]
float64 phi_signed  # signed certainty interval phi(+/-)
string label        # logical state (12 regions of the Para-Analyzer)
float64 decision    # 1.0 true, 0.0 false, 0.5 undefined
