/*
 * Filter Coefficients (C Source) generated by the Filter Design and Analysis Tool
 * Generated by MATLAB(R) 9.13 and Signal Processing Toolbox 9.1.
 * Generated on: 22-Nov-2024 16:51:30
 */

/*
 * Discrete-Time FIR Filter (real)
 * -------------------------------
 * Filter Structure  : Direct-Form FIR
 * Filter Length     : 305
 * Stable            : Yes
 * Linear Phase      : Yes (Type 1)
 */

/* General type conversion for MATLAB generated C-code  */
#include "tmwtypes.h"
/* 
 * Expected path to tmwtypes.h 
 * /Applications/MATLAB_R2022b.app/extern/include/tmwtypes.h 
 */
const int BPL = 305;
const real64_T BP[305] = {
  -3.413924890067e-05,-0.0001063660345381,9.608678543518e-05,-8.344138986944e-05,
  3.699017290274e-05, 4.09852583552e-05,-0.0001247317868527,0.0001761043320756,
  -0.0001614376427014,6.965607686792e-05,7.659457688491e-05,-0.0002237405938422,
   0.000305630698458,-0.000271782074657, 0.000113902775119, 0.000122232719976,
  -0.0003477932471668,0.0004635571343209,-0.0004025913055232,0.0001648593087406,
  0.0001732355815645,-0.0004821355153609,0.0006289632297444,-0.0005347640240907,
  0.0002143708178974,0.0002206701110527,-0.0006010636340644,0.0007672782451822,
  -0.0006380694411654,0.0002500319058007,0.0002513131216888,-0.0006679043199236,
  0.0008307547576489,-0.0006720912496918,0.0002557915952597,0.0002485520559898,
  -0.0006378708670232,0.0007626947827116,-0.0005900362599415,0.0002134597433922,
  0.0001942170143847,-0.0004631451049028,0.0005042291066993,-0.000344886087998,
  0.0001055000959827,7.111093089082e-05,-0.0001005194564399, 4.46343777094e-06,
  0.0001022198001334,-8.197031312086e-05,-0.0001332778189935,0.0004794142635662,
  -0.0007680736626787,0.0007723087413808,-0.0003551128044117,-0.0004231672797246,
   0.001282255884496,-0.001812981058279, 0.001658916246607,-0.0007089394711813,
  -0.000791095277198, 0.002280787610073,-0.003087688944412, 0.002719844715725,
  -0.001124352899428,-0.001214811416858, 0.003409170821469,-0.004500418550061,
   0.003872702044054,-0.001566728758265,-0.001656647886917, 0.004560468415285,
  -0.005909256173058, 0.004994889747981,-0.001986351110596,-0.002064169795177,
   0.005590486594065,-0.007127898702125, 0.005929585352104,-0.002321148730739,
  -0.002373464122051, 0.006327328062295,-0.007939109120685,  0.00649785672712,
  -0.002501830686971,-0.002514591247032, 0.006586965964439,-0.008115218019355,
   0.006516213862265,-0.002458892090569,-0.002419026142324, 0.006193283934366,
  -0.007443828789871, 0.005818006233313,-0.002131010832366, -0.00202815340615,
   0.005000781085721,-0.005756032717408, 0.004276352962186,-0.001473794341891,
  -0.001301945038092, 0.002916991031148,-0.002953715872146, 0.001825679079832,
  -0.0004677250271534,-0.0002266577583863,-7.835767112964e-05,0.0009680994898897,
  -0.001521311614918,0.0008757852137538, 0.001179709601458, -0.00392044932875,
    0.00590722963383,-0.005664044880317,  0.00251144018547, 0.002865117107211,
  -0.008455178106872,  0.01165162210804, -0.01041372875045, 0.004360719806074,
   0.004744858247429, -0.01344510881983,  0.01788907546217, -0.01550349226269,
   0.006316492995535,  0.00670703605286, -0.01858549856815,  0.02422972172694,
   -0.02060829531198, 0.008251359907473, 0.008621515486091, -0.02352958939218,
    0.03023906070305, -0.02537318840967,  0.01002893787027,  0.01035167684983,
   -0.02792036909994,   0.0354784155765, -0.02944678125175,  0.01151690089712,
    0.01176758589801, -0.03142542620613,  0.03954808328331, -0.03251614159364,
      0.012600188662,  0.01275911817847, -0.03377060103262,  0.04212817086617,
   -0.03433887782662,  0.01319290589706,  0.01324734980265, -0.03476861550384,
    0.04301201111015, -0.03476861550384,  0.01324734980265,  0.01319290589706,
   -0.03433887782662,  0.04212817086617, -0.03377060103262,  0.01275911817847,
      0.012600188662, -0.03251614159364,  0.03954808328331, -0.03142542620613,
    0.01176758589801,  0.01151690089712, -0.02944678125175,   0.0354784155765,
   -0.02792036909994,  0.01035167684983,  0.01002893787027, -0.02537318840967,
    0.03023906070305, -0.02352958939218, 0.008621515486091, 0.008251359907473,
   -0.02060829531198,  0.02422972172694, -0.01858549856815,  0.00670703605286,
   0.006316492995535, -0.01550349226269,  0.01788907546217, -0.01344510881983,
   0.004744858247429, 0.004360719806074, -0.01041372875045,  0.01165162210804,
  -0.008455178106872, 0.002865117107211,  0.00251144018547,-0.005664044880317,
    0.00590722963383, -0.00392044932875, 0.001179709601458,0.0008757852137538,
  -0.001521311614918,0.0009680994898897,-7.835767112964e-05,-0.0002266577583863,
  -0.0004677250271534, 0.001825679079832,-0.002953715872146, 0.002916991031148,
  -0.001301945038092,-0.001473794341891, 0.004276352962186,-0.005756032717408,
   0.005000781085721, -0.00202815340615,-0.002131010832366, 0.005818006233313,
  -0.007443828789871, 0.006193283934366,-0.002419026142324,-0.002458892090569,
   0.006516213862265,-0.008115218019355, 0.006586965964439,-0.002514591247032,
  -0.002501830686971,  0.00649785672712,-0.007939109120685, 0.006327328062295,
  -0.002373464122051,-0.002321148730739, 0.005929585352104,-0.007127898702125,
   0.005590486594065,-0.002064169795177,-0.001986351110596, 0.004994889747981,
  -0.005909256173058, 0.004560468415285,-0.001656647886917,-0.001566728758265,
   0.003872702044054,-0.004500418550061, 0.003409170821469,-0.001214811416858,
  -0.001124352899428, 0.002719844715725,-0.003087688944412, 0.002280787610073,
  -0.000791095277198,-0.0007089394711813, 0.001658916246607,-0.001812981058279,
   0.001282255884496,-0.0004231672797246,-0.0003551128044117,0.0007723087413808,
  -0.0007680736626787,0.0004794142635662,-0.0001332778189935,-8.197031312086e-05,
  0.0001022198001334, 4.46343777094e-06,-0.0001005194564399,7.111093089082e-05,
  0.0001055000959827,-0.000344886087998,0.0005042291066993,-0.0004631451049028,
  0.0001942170143847,0.0002134597433922,-0.0005900362599415,0.0007626947827116,
  -0.0006378708670232,0.0002485520559898,0.0002557915952597,-0.0006720912496918,
  0.0008307547576489,-0.0006679043199236,0.0002513131216888,0.0002500319058007,
  -0.0006380694411654,0.0007672782451822,-0.0006010636340644,0.0002206701110527,
  0.0002143708178974,-0.0005347640240907,0.0006289632297444,-0.0004821355153609,
  0.0001732355815645,0.0001648593087406,-0.0004025913055232,0.0004635571343209,
  -0.0003477932471668, 0.000122232719976, 0.000113902775119,-0.000271782074657,
   0.000305630698458,-0.0002237405938422,7.659457688491e-05,6.965607686792e-05,
  -0.0001614376427014,0.0001761043320756,-0.0001247317868527, 4.09852583552e-05,
  3.699017290274e-05,-8.344138986944e-05,9.608678543518e-05,-0.0001063660345381,
  -3.413924890067e-05
};
