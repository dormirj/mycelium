#pragma once

#include "daisy.h"
// #include "CustomPages.h"
#include "daisy_mycelium.h"

#define PIN_OLED_DC 9
#define PIN_OLED_RESET 30

namespace daisy
{
void ClearDisplay(const daisy::UiCanvasDescriptor& canvas)
{
}

void FlushDisplay(const daisy::UiCanvasDescriptor& canvas)
{
}

class TorusPatchUI : public UI
{
  public:
    ~TorusPatchUI(){};

    void InitDisplay(DaisyMycelium* hw)
    {

    }

    void DoEvents()
    {

    }

    void GenerateEvents() {}

  private:

};

} // namespace daisy