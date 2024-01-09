// Copyright 2015 Emilie Gillet.
//
// Author: Emilie Gillet (emilie.o.gillet@gmail.com)
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//
// See http://creativecommons.org/licenses/MIT/ for more information.
//
// -----------------------------------------------------------------------------
//
// Resources definitions.
//
// Automatically generated with:
// make resources

// This is the only thing we'd need DaisySP for, so I'll just define it here
#define PI_F 3.1415927410125732421875f
#define TWOPI_F (2.0f * PI_F)

#include "resources.h"
#include <math.h>

namespace mycelium
{
void InitResources()
{
    // lut_sine
    for(int i = 0; i < LUT_SINE_SIZE; i++)
    {
        float t     = (float)i / 4096.f * TWOPI_F;
        lut_sine[i] = sin(t);
    }
}

const int16_t* lookup_table_int16_table[] = {};


const uint32_t* lookup_table_uint32_table[] = {};

float lut_sine[LUT_SINE_SIZE];

const float lut_4_decades[] = {
    1.000000000e+00, 1.036632928e+00, 1.074607828e+00, 1.113973860e+00,
    1.154781985e+00, 1.197085030e+00, 1.240937761e+00, 1.286396945e+00,
    1.333521432e+00, 1.382372227e+00, 1.433012570e+00, 1.485508017e+00,
    1.539926526e+00, 1.596338544e+00, 1.654817100e+00, 1.715437896e+00,
    1.778279410e+00, 1.843422992e+00, 1.910952975e+00, 1.980956779e+00,
    2.053525026e+00, 2.128751662e+00, 2.206734069e+00, 2.287573200e+00,
    2.371373706e+00, 2.458244069e+00, 2.548296748e+00, 2.641648320e+00,
    2.738419634e+00, 2.838735965e+00, 2.942727176e+00, 3.050527890e+00,
    3.162277660e+00, 3.278121151e+00, 3.398208329e+00, 3.522694651e+00,
    3.651741273e+00, 3.785515249e+00, 3.924189758e+00, 4.067944321e+00,
    4.216965034e+00, 4.371444813e+00, 4.531583638e+00, 4.697588817e+00,
    4.869675252e+00, 5.048065717e+00, 5.232991147e+00, 5.424690937e+00,
    5.623413252e+00, 5.829415347e+00, 6.042963902e+00, 6.264335367e+00,
    6.493816316e+00, 6.731703824e+00, 6.978305849e+00, 7.233941627e+00,
    7.498942093e+00, 7.773650302e+00, 8.058421878e+00, 8.353625470e+00,
    8.659643234e+00, 8.976871324e+00, 9.305720409e+00, 9.646616199e+00,
    1.000000000e+01, 1.036632928e+01, 1.074607828e+01, 1.113973860e+01,
    1.154781985e+01, 1.197085030e+01, 1.240937761e+01, 1.286396945e+01,
    1.333521432e+01, 1.382372227e+01, 1.433012570e+01, 1.485508017e+01,
    1.539926526e+01, 1.596338544e+01, 1.654817100e+01, 1.715437896e+01,
    1.778279410e+01, 1.843422992e+01, 1.910952975e+01, 1.980956779e+01,
    2.053525026e+01, 2.128751662e+01, 2.206734069e+01, 2.287573200e+01,
    2.371373706e+01, 2.458244069e+01, 2.548296748e+01, 2.641648320e+01,
    2.738419634e+01, 2.838735965e+01, 2.942727176e+01, 3.050527890e+01,
    3.162277660e+01, 3.278121151e+01, 3.398208329e+01, 3.522694651e+01,
    3.651741273e+01, 3.785515249e+01, 3.924189758e+01, 4.067944321e+01,
    4.216965034e+01, 4.371444813e+01, 4.531583638e+01, 4.697588817e+01,
    4.869675252e+01, 5.048065717e+01, 5.232991147e+01, 5.424690937e+01,
    5.623413252e+01, 5.829415347e+01, 6.042963902e+01, 6.264335367e+01,
    6.493816316e+01, 6.731703824e+01, 6.978305849e+01, 7.233941627e+01,
    7.498942093e+01, 7.773650302e+01, 8.058421878e+01, 8.353625470e+01,
    8.659643234e+01, 8.976871324e+01, 9.305720409e+01, 9.646616199e+01,
    1.000000000e+02, 1.036632928e+02, 1.074607828e+02, 1.113973860e+02,
    1.154781985e+02, 1.197085030e+02, 1.240937761e+02, 1.286396945e+02,
    1.333521432e+02, 1.382372227e+02, 1.433012570e+02, 1.485508017e+02,
    1.539926526e+02, 1.596338544e+02, 1.654817100e+02, 1.715437896e+02,
    1.778279410e+02, 1.843422992e+02, 1.910952975e+02, 1.980956779e+02,
    2.053525026e+02, 2.128751662e+02, 2.206734069e+02, 2.287573200e+02,
    2.371373706e+02, 2.458244069e+02, 2.548296748e+02, 2.641648320e+02,
    2.738419634e+02, 2.838735965e+02, 2.942727176e+02, 3.050527890e+02,
    3.162277660e+02, 3.278121151e+02, 3.398208329e+02, 3.522694651e+02,
    3.651741273e+02, 3.785515249e+02, 3.924189758e+02, 4.067944321e+02,
    4.216965034e+02, 4.371444813e+02, 4.531583638e+02, 4.697588817e+02,
    4.869675252e+02, 5.048065717e+02, 5.232991147e+02, 5.424690937e+02,
    5.623413252e+02, 5.829415347e+02, 6.042963902e+02, 6.264335367e+02,
    6.493816316e+02, 6.731703824e+02, 6.978305849e+02, 7.233941627e+02,
    7.498942093e+02, 7.773650302e+02, 8.058421878e+02, 8.353625470e+02,
    8.659643234e+02, 8.976871324e+02, 9.305720409e+02, 9.646616199e+02,
    1.000000000e+03, 1.036632928e+03, 1.074607828e+03, 1.113973860e+03,
    1.154781985e+03, 1.197085030e+03, 1.240937761e+03, 1.286396945e+03,
    1.333521432e+03, 1.382372227e+03, 1.433012570e+03, 1.485508017e+03,
    1.539926526e+03, 1.596338544e+03, 1.654817100e+03, 1.715437896e+03,
    1.778279410e+03, 1.843422992e+03, 1.910952975e+03, 1.980956779e+03,
    2.053525026e+03, 2.128751662e+03, 2.206734069e+03, 2.287573200e+03,
    2.371373706e+03, 2.458244069e+03, 2.548296748e+03, 2.641648320e+03,
    2.738419634e+03, 2.838735965e+03, 2.942727176e+03, 3.050527890e+03,
    3.162277660e+03, 3.278121151e+03, 3.398208329e+03, 3.522694651e+03,
    3.651741273e+03, 3.785515249e+03, 3.924189758e+03, 4.067944321e+03,
    4.216965034e+03, 4.371444813e+03, 4.531583638e+03, 4.697588817e+03,
    4.869675252e+03, 5.048065717e+03, 5.232991147e+03, 5.424690937e+03,
    5.623413252e+03, 5.829415347e+03, 6.042963902e+03, 6.264335367e+03,
    6.493816316e+03, 6.731703824e+03, 6.978305849e+03, 7.233941627e+03,
    7.498942093e+03, 7.773650302e+03, 8.058421878e+03, 8.353625470e+03,
    8.659643234e+03, 8.976871324e+03, 9.305720409e+03, 9.646616199e+03,
    1.000000000e+04,
};
const float lut_svf_shift[] = {
    2.500000000e-01, 2.408119579e-01, 2.316544611e-01, 2.225575501e-01,
    2.135502761e-01, 2.046602549e-01, 1.959132760e-01, 1.873329789e-01,
    1.789406032e-01, 1.707548172e-01, 1.627916233e-01, 1.550643347e-01,
    1.475836177e-01, 1.403575876e-01, 1.333919506e-01, 1.266901772e-01,
    1.202537001e-01, 1.140821254e-01, 1.081734480e-01, 1.025242668e-01,
    9.712999179e-02, 9.198504051e-02, 8.708302003e-02, 8.241689360e-02,
    7.797913038e-02, 7.376183852e-02, 6.975688172e-02, 6.595598018e-02,
    6.235079694e-02, 5.893301078e-02, 5.569437701e-02, 5.262677742e-02,
    4.972226058e-02, 4.697307381e-02, 4.437168789e-02, 4.191081545e-02,
    3.958342416e-02, 3.738274529e-02, 3.530227864e-02, 3.333579426e-02,
    3.147733169e-02, 2.972119704e-02, 2.806195849e-02, 2.649444041e-02,
    2.501371653e-02, 2.361510230e-02, 2.229414676e-02, 2.104662398e-02,
    1.986852431e-02, 1.875604550e-02, 1.770558386e-02, 1.671372543e-02,
    1.577723728e-02, 1.489305906e-02, 1.405829467e-02, 1.327020425e-02,
    1.252619642e-02, 1.182382076e-02, 1.116076060e-02, 1.053482614e-02,
    9.943947824e-03, 9.386169992e-03, 8.859644866e-03, 8.362626781e-03,
    7.893466717e-03, 7.450607078e-03, 7.032576744e-03, 6.637986365e-03,
    6.265523903e-03, 5.913950392e-03, 5.582095932e-03, 5.268855886e-03,
    4.973187279e-03, 4.694105394e-03, 4.430680542e-03, 4.182035018e-03,
    3.947340207e-03, 3.725813861e-03, 3.516717519e-03, 3.319354065e-03,
    3.133065427e-03, 2.957230396e-03, 2.791262569e-03, 2.634608406e-03,
    2.486745394e-03, 2.347180309e-03, 2.215447582e-03, 2.091107747e-03,
    1.973745986e-03, 1.862970740e-03, 1.758412418e-03, 1.659722154e-03,
    1.566570656e-03, 1.478647104e-03, 1.395658114e-03, 1.317326764e-03,
    1.243391669e-03, 1.173606108e-03, 1.107737206e-03, 1.045565155e-03,
    9.868824789e-04, 9.314933471e-04, 8.792129165e-04, 8.298667176e-04,
    7.832900713e-04, 7.393275405e-04, 6.978324110e-04, 6.586662024e-04,
    6.216982059e-04, 5.868050482e-04, 5.538702800e-04, 5.227839874e-04,
    4.934424252e-04, 4.657476707e-04, 4.396072968e-04, 4.149340639e-04,
    3.916456285e-04, 3.696642688e-04, 3.489166247e-04, 3.293334538e-04,
    3.108493994e-04, 2.934027734e-04, 2.769353496e-04, 2.613921700e-04,
    2.467213608e-04, 2.328739600e-04, 2.198037532e-04, 2.074671201e-04,
    1.958228884e-04, 1.848321967e-04, 1.744583648e-04, 1.646667709e-04,
    1.554247368e-04, 1.467014179e-04, 1.384677010e-04, 1.306961070e-04,
    1.233606989e-04, 1.164369956e-04, 1.099018897e-04, 1.037335710e-04,
    9.791145345e-05, 9.241610615e-05, 8.722918894e-05, 8.233339098e-05,
    7.771237301e-05, 7.335071282e-05, 6.923385378e-05, 6.534805627e-05,
    6.168035179e-05, 5.821849973e-05, 5.495094649e-05, 5.186678690e-05,
    4.895572788e-05, 4.620805405e-05, 4.361459529e-05, 4.116669618e-05,
    3.885618709e-05, 3.667535690e-05, 3.461692730e-05, 3.267402848e-05,
    3.084017618e-05, 2.910925011e-05, 2.747547345e-05, 2.593339362e-05,
    2.447786409e-05, 2.310402715e-05, 2.180729775e-05, 2.058334818e-05,
    1.942809362e-05, 1.833767851e-05, 1.730846370e-05, 1.633701428e-05,
    1.542008813e-05, 1.455462508e-05, 1.373773675e-05, 1.296669683e-05,
    1.223893206e-05, 1.155201359e-05, 1.090364889e-05, 1.029167410e-05,
    9.714046817e-06, 9.168839263e-06, 8.654231857e-06, 8.168507146e-06,
    7.710044069e-06, 7.277312546e-06, 6.868868378e-06, 6.483348419e-06,
    6.119466033e-06, 5.776006796e-06, 5.451824445e-06, 5.145837051e-06,
    4.857023409e-06, 4.584419632e-06, 4.327115929e-06, 4.084253574e-06,
    3.855022035e-06, 3.638656274e-06, 3.434434189e-06, 3.241674210e-06,
    3.059733017e-06, 2.888003398e-06, 2.725912223e-06, 2.572918525e-06,
    2.428511705e-06, 2.292209816e-06, 2.163557965e-06, 2.042126787e-06,
    1.927511018e-06, 1.819328137e-06, 1.717217095e-06, 1.620837105e-06,
    1.529866508e-06, 1.444001699e-06, 1.362956111e-06, 1.286459263e-06,
    1.214255852e-06, 1.146104908e-06, 1.081778982e-06, 1.021063394e-06,
    9.637555088e-07, 9.096640684e-07, 8.586085474e-07, 8.104185525e-07,
    7.649332542e-07, 7.220008496e-07, 6.814780557e-07, 6.432296314e-07,
    6.071279262e-07, 5.730524541e-07, 5.408894912e-07, 5.105316968e-07,
    4.818777544e-07, 4.548320342e-07, 4.293042737e-07, 4.052092763e-07,
    3.824666271e-07, 3.610004248e-07, 3.407390278e-07, 3.216148157e-07,
    3.035639631e-07, 2.865262270e-07, 2.704447456e-07, 2.552658484e-07,
    2.409388772e-07, 2.274160171e-07, 2.146521368e-07, 2.026046381e-07,
    1.912333136e-07, 1.805002124e-07, 1.703695139e-07, 1.608074078e-07,
    1.517819816e-07, 1.432631135e-07, 1.352223728e-07, 1.276329242e-07,
    1.204694386e-07,
};
const float lut_stiffness[] = {
    -6.250000000e-02, -6.152343750e-02, -6.054687500e-02, -5.957031250e-02,
    -5.859375000e-02, -5.761718750e-02, -5.664062500e-02, -5.566406250e-02,
    -5.468750000e-02, -5.371093750e-02, -5.273437500e-02, -5.175781250e-02,
    -5.078125000e-02, -4.980468750e-02, -4.882812500e-02, -4.785156250e-02,
    -4.687500000e-02, -4.589843750e-02, -4.492187500e-02, -4.394531250e-02,
    -4.296875000e-02, -4.199218750e-02, -4.101562500e-02, -4.003906250e-02,
    -3.906250000e-02, -3.808593750e-02, -3.710937500e-02, -3.613281250e-02,
    -3.515625000e-02, -3.417968750e-02, -3.320312500e-02, -3.222656250e-02,
    -3.125000000e-02, -3.027343750e-02, -2.929687500e-02, -2.832031250e-02,
    -2.734375000e-02, -2.636718750e-02, -2.539062500e-02, -2.441406250e-02,
    -2.343750000e-02, -2.246093750e-02, -2.148437500e-02, -2.050781250e-02,
    -1.953125000e-02, -1.855468750e-02, -1.757812500e-02, -1.660156250e-02,
    -1.562500000e-02, -1.464843750e-02, -1.367187500e-02, -1.269531250e-02,
    -1.171875000e-02, -1.074218750e-02, -9.765625000e-03, -8.789062500e-03,
    -7.812500000e-03, -6.835937500e-03, -5.859375000e-03, -4.882812500e-03,
    -3.906250000e-03, -2.929687500e-03, -1.953125000e-03, -9.765625000e-04,
    0.000000000e+00,  0.000000000e+00,  0.000000000e+00,  0.000000000e+00,
    0.000000000e+00,  0.000000000e+00,  0.000000000e+00,  0.000000000e+00,
    0.000000000e+00,  0.000000000e+00,  0.000000000e+00,  0.000000000e+00,
    0.000000000e+00,  6.029410294e-05,  3.672617230e-04,  6.835957809e-04,
    1.009582073e-03,  1.345515115e-03,  1.691698412e-03,  2.048444725e-03,
    2.416076364e-03,  2.794925468e-03,  3.185334315e-03,  3.587655624e-03,
    4.002252878e-03,  4.429500650e-03,  4.869784943e-03,  5.323503537e-03,
    5.791066350e-03,  6.272895808e-03,  6.769427226e-03,  7.281109202e-03,
    7.808404022e-03,  8.351788076e-03,  8.911752293e-03,  9.488802580e-03,
    1.008346028e-02,  1.069626264e-02,  1.132776331e-02,  1.197853283e-02,
    1.264915914e-02,  1.334024813e-02,  1.405242417e-02,  1.478633069e-02,
    1.554263074e-02,  1.632200761e-02,  1.712516545e-02,  1.795282987e-02,
    1.880574864e-02,  1.968469234e-02,  2.059045506e-02,  2.152385512e-02,
    2.248573583e-02,  2.347696619e-02,  2.449844176e-02,  2.555108540e-02,
    2.663584813e-02,  2.775370999e-02,  2.890568094e-02,  3.009280173e-02,
    3.131614488e-02,  3.257681565e-02,  3.387595299e-02,  3.521473064e-02,
    3.659435812e-02,  3.801608189e-02,  3.948118641e-02,  4.099099536e-02,
    4.254687278e-02,  4.415022437e-02,  4.580249868e-02,  4.750518848e-02,
    4.925983210e-02,  5.106801479e-02,  5.293137017e-02,  5.485158172e-02,
    5.683038428e-02,  5.886956562e-02,  6.097096806e-02,  6.313649016e-02,
    6.536808837e-02,  6.766777886e-02,  7.003763933e-02,  7.247981084e-02,
    7.499649981e-02,  7.758997998e-02,  8.026259446e-02,  8.301675786e-02,
    8.585495846e-02,  8.877976048e-02,  9.179380636e-02,  9.489981918e-02,
    9.810060511e-02,  1.013990559e-01,  1.047981517e-01,  1.083009634e-01,
    1.119106556e-01,  1.156304895e-01,  1.194638260e-01,  1.234141283e-01,
    1.274849653e-01,  1.316800149e-01,  1.360030671e-01,  1.404580277e-01,
    1.450489216e-01,  1.497798965e-01,  1.546552266e-01,  1.596793166e-01,
    1.648567056e-01,  1.701920711e-01,  1.756902336e-01,  1.813561603e-01,
    1.871949702e-01,  1.932119385e-01,  1.994125013e-01,  2.058022605e-01,
    2.123869891e-01,  2.191726361e-01,  2.261653322e-01,  2.333713949e-01,
    2.407973346e-01,  2.484498605e-01,  2.563358863e-01,  2.644625367e-01,
    2.728371538e-01,  2.814673039e-01,  2.903607839e-01,  2.995256288e-01,
    3.089701187e-01,  3.187027863e-01,  3.287324247e-01,  3.390680953e-01,
    3.497191360e-01,  3.606951697e-01,  3.720061128e-01,  3.836621843e-01,
    3.956739150e-01,  4.080521572e-01,  4.208080940e-01,  4.339532500e-01,
    4.474995013e-01,  4.614590865e-01,  4.758446177e-01,  4.906690914e-01,
    5.059459012e-01,  5.216888491e-01,  5.379121581e-01,  5.546304856e-01,
    5.718589358e-01,  5.896130741e-01,  6.079089407e-01,  6.267630651e-01,
    6.461924814e-01,  6.662147434e-01,  6.868479405e-01,  7.081107139e-01,
    7.300222738e-01,  7.526024164e-01,  7.758715422e-01,  7.998506739e-01,
    8.245614757e-01,  8.500262730e-01,  8.762680723e-01,  9.033105820e-01,
    9.311782340e-01,  9.598962059e-01,  9.894904431e-01,  1.000000745e+00,
    1.000037649e+00,  1.000262504e+00,  1.000964607e+00,  1.002570034e+00,
    1.005639154e+00,  1.010861180e+00,  1.019043988e+00,  1.031097087e+00,
    1.048005353e+00,  1.070791059e+00,  1.100461817e+00,  1.137942574e+00,
    1.183990632e+00,  1.239094135e+00,  1.303356514e+00,  1.376372085e+00,
    1.457101344e+00,  1.543758274e+00,  1.633725943e+00,  1.723520185e+00,
    1.808823654e+00,  1.884612937e+00,  1.945398753e+00,  2.000000000e+00,
    2.000000000e+00,
};
const float lut_fm_frequency_quantizer[] = {
    -1.200000000e+01, -1.200000000e+01, -1.200000000e+01, -1.184000000e+01,
    -1.184000000e+01, -1.184000000e+01, -1.111000000e+01, -1.038000000e+01,
    -9.650000000e+00, -8.920000000e+00, -8.190000000e+00, -7.460000000e+00,
    -6.730000000e+00, -6.000000000e+00, -6.000000000e+00, -6.000000000e+00,
    -5.545511612e+00, -5.091023223e+00, -4.636534835e+00, -4.182046446e+00,
    -4.182046446e+00, -4.182046446e+00, -3.659290641e+00, -3.136534835e+00,
    -2.613779029e+00, -2.091023223e+00, -1.568267417e+00, -1.045511612e+00,
    -5.227558058e-01, 0.000000000e+00,  0.000000000e+00,  0.000000000e+00,
    1.600000000e-01,  1.600000000e-01,  1.600000000e-01,  8.900000000e-01,
    1.620000000e+00,  2.350000000e+00,  3.080000000e+00,  3.810000000e+00,
    4.540000000e+00,  5.270000000e+00,  6.000000000e+00,  6.000000000e+00,
    6.000000000e+00,  6.454488388e+00,  6.908976777e+00,  7.363465165e+00,
    7.817953554e+00,  7.817953554e+00,  7.817953554e+00,  8.285529931e+00,
    8.753106309e+00,  9.220682687e+00,  9.688259065e+00,  9.688259065e+00,
    9.688259065e+00,  1.026619430e+01,  1.084412953e+01,  1.142206477e+01,
    1.200000000e+01,  1.200000000e+01,  1.200000000e+01,  1.216000000e+01,
    1.216000000e+01,  1.216000000e+01,  1.262977500e+01,  1.309955001e+01,
    1.356932501e+01,  1.403910002e+01,  1.403910002e+01,  1.403910002e+01,
    1.490761987e+01,  1.577613972e+01,  1.664465957e+01,  1.751317942e+01,
    1.751317942e+01,  1.751317942e+01,  1.800000000e+01,  1.800000000e+01,
    1.800000000e+01,  1.850977500e+01,  1.901955001e+01,  1.901955001e+01,
    1.901955001e+01,  1.981795355e+01,  1.981795355e+01,  1.981795355e+01,
    2.066386428e+01,  2.150977500e+01,  2.150977500e+01,  2.150977500e+01,
    2.213233125e+01,  2.275488750e+01,  2.337744375e+01,  2.400000000e+01,
    2.400000000e+01,  2.400000000e+01,  2.450977500e+01,  2.501955001e+01,
    2.501955001e+01,  2.501955001e+01,  2.547403840e+01,  2.592852679e+01,
    2.638301517e+01,  2.683750356e+01,  2.683750356e+01,  2.683750356e+01,
    2.735032035e+01,  2.786313714e+01,  2.786313714e+01,  2.786313714e+01,
    2.839735285e+01,  2.893156857e+01,  2.946578428e+01,  3.000000000e+01,
    3.000000000e+01,  3.000000000e+01,  3.075000000e+01,  3.150000000e+01,
    3.225000000e+01,  3.300000000e+01,  3.375000000e+01,  3.450000000e+01,
    3.525000000e+01,  3.600000000e+01,  3.600000000e+01,  3.600000000e+01,
    3.600000000e+01,
};


const float* lookup_table_table[] = {
    lut_sine,
    lut_4_decades,
    lut_svf_shift,
    lut_stiffness,
    lut_fm_frequency_quantizer,
};


} // namespace mycelium