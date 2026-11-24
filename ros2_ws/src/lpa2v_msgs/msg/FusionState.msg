# State of the paraconsistent fusion (NAP + ParaExtrctr network) of N redundant sensors
std_msgs/Header header
float64 value           # fused value (physical unit)
float64 mu_group        # muE of the group of accepted sensors (ParaExtrctr output, in the UD)
float64 certainty       # min(phiE) of the accepted sensors (AND connective); 0 = indefiniteness
float64 contradiction   # max |Gct| between sensors and group (OR connective)
bool indefinite         # no decision possible; value = last defined output
float64[] mu            # evidence degree of each sensor in the UD
float64[] gct           # contradiction degree of each sensor against the group
float64[] phi_e         # evidence interval of each sensor (1 - |Gct|)
string[] labels         # "t" consistent, "⊤"/"⊥" contradiction
int32[] rejected        # indices of the rejected sensors
