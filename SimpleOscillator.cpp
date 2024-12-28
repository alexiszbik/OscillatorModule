#include "daisy_patch_sm.h"
#include "daisysp.h"
#include "SmoothValue.h"

using namespace daisy;
using namespace patch_sm;
using namespace daisysp;

Switch toggle;
ydaisy::SmoothValue glider;

struct Osc {
    static const int subOscCount = 2;      

    VariableShapeOscillator osc[subOscCount];
    float currentFreq[subOscCount];

    bool useExtraOsc = false;
    float extraOscTune = 0;
    
    void init(double sampleRate) {
        for (int i = 0; i < subOscCount; i++) {
            osc[i].Init(sampleRate);
            osc[i].SetSync(1);
        }
    }
    
    void setMidi(float midiValue) {
        currentFreq[0] = mtof(midiValue);
        currentFreq[1] = mtof(midiValue + extraOscTune);
        for (int i = 0; i < subOscCount; i++) {
            osc[i].SetFreq(currentFreq[i]);
        }
        
    }

    void setShape(float shapeValue) {
        for (int i = 0; i < subOscCount; i++) {
            osc[i].SetPW(fmax(shapeValue, 0.5));
            osc[i].SetWaveshape(fmin(shapeValue + 0.5, 1));
        }
    }

    void setSync(float syncValue) {
        for (int i = 0; i < subOscCount; i++) {
            osc[i].SetSyncFreq(currentFreq[i] + currentFreq[i] * fmax(syncValue, 0) * 8);
        }
    }

    void setGroupValue(float value) {
        if (value >= 0.1) {
            useExtraOsc = true;
            float curve = (value - 0.1)/0.9;
            if (curve <= 0.25) {
                extraOscTune = curve * 2;
            } else if (curve <= 0.5) {
                extraOscTune = 3;
            } else if (curve <= 0.75) {
                extraOscTune = 4;
            } else if (curve <= 1) {
                extraOscTune = 5;
            }
        } else {
            useExtraOsc = false;
            extraOscTune = 0;
        }
    }

    float process() {
        if (useExtraOsc) {
            return (osc[0].Process() + osc[1].Process()) * 0.9;
        } else {
            return osc[0].Process();
        }
        
    }
};

DaisyPatchSM patch;
static const int oscCount = 2;
Osc   osc[oscCount] = {Osc(), Osc()};

uint32_t timer;
int previousRoundPitch = 0;
bool ledStatus = false;

float getBTune() {
    float bTuneRatio = fmap(patch.GetAdcValue(ADC_9), -1.f, 1.f);
    float absTuneRatio = fabs(bTuneRatio);
    
    if (absTuneRatio <= 0.333) {
        bTuneRatio = bTuneRatio/0.333;
    } else if (absTuneRatio <= 0.666) {
        bTuneRatio = bTuneRatio > 0 ? 7 : -5;
    } else {
        bTuneRatio = bTuneRatio > 0 ? 12 : -12;
    }
    return bTuneRatio;
}

void processOscParameters(Osc* osc, float shapeValue, float syncValue, float groupValue) {
    osc->setShape(shapeValue);
    osc->setSync(syncValue);
    osc->setGroupValue(groupValue);
}

void AudioCallback(AudioHandle::InputBuffer  in,
                   AudioHandle::OutputBuffer out,
                   size_t                    size)
{
    toggle.Debounce();

    /** Get the current toggle state */
    bool state = toggle.Pressed();

    for(size_t i = 0; i < size; i++)
    {
        patch.ProcessAllControls();

        float pitch = patch.GetAdcValue(CV_5);
        pitch = pitch > 0.05 ? pitch : 0;
        pitch = fmap(pitch, 0, 24.f);

        float v1 = (patch.GetAdcValue(CV_1) + 0.02708) * 0.97 * 60;
        int roundPitch = roundf(v1);
        if (roundPitch != previousRoundPitch) {
            previousRoundPitch = roundPitch;
            ledStatus = true;
        }

        glider.setValue(roundPitch);

        float glideTime = fmax(patch.GetAdcValue(ADC_10), 0);

        glider.dezipperCheck(44100 * glideTime * glideTime);

        float v2 = (patch.GetAdcValue(CV_2) + 0.0226) * 0.98 * 60;
        float v3 = (patch.GetAdcValue(CV_3) + 0.02855) * 0.97 * 60;

        float v4 = (patch.GetAdcValue(CV_4) + 0.0225);
        float v8 = (patch.GetAdcValue(CV_8) + 0.0231);

        v2 = fabs(v2) > 0.2 ? v2 : 0;
        v3 = fabs(v3) > 0.2 ? v3 : 0;

        float midiA = fclamp(36 + pitch + glider.getAndStep() + v2 + v3, 0.f, 127.f);
        float midiB = fclamp(midiA + getBTune(), 0.f, 127.f);

        osc[0].setMidi(midiA);
        osc[1].setMidi(midiB);

        float syncValue = fmap(patch.GetAdcValue(CV_6), 0, 1.f) + v4 + v8;
        float shapeA = fmap(patch.GetAdcValue(ADC_11), 0, 1.f);
        float shapeB = fmap(patch.GetAdcValue(ADC_12), 0, 1.f);

        float groupValue = fclamp(patch.GetAdcValue(CV_7), 0, 1.f);

        processOscParameters(&osc[0], shapeA, state ? syncValue : 0, groupValue);
        processOscParameters(&osc[1], shapeB, state ? 0 : syncValue, groupValue);

        OUT_L[i]  = osc[0].process();;
        OUT_R[i]  = osc[1].process();
    }
}

int main(void)
{
    patch.Init();

    toggle.Init(patch.B8);

    auto led = new dsy_gpio();
    led->pin  = patch.D1;
    led->mode = DSY_GPIO_MODE_OUTPUT_PP;
    led->pull = DSY_GPIO_NOPULL;
    dsy_gpio_init(led);

    timer = System::GetNow();

    for (int i = 0; i < oscCount; i++) {
        osc[i].init(patch.AudioSampleRate());
    }

    patch.StartAudio(AudioCallback);
    while(1) {
        auto newTime = System::GetNow();
        if (ledStatus) {
             dsy_gpio_write(led, true);
             ledStatus = false;
             timer = newTime;
        } 
        if ((newTime - timer) > 20) {
             dsy_gpio_write(led, false);
        }
         
    }
}
