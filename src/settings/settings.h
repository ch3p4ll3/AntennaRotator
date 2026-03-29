#pragma once

#include <Arduino.h>
#include <Preferences.h>


struct SettingsData {
    float range;
    float offset;
    float stepsPerDegree;
};


constexpr SettingsData DEFAULT_CONFIGS = {
    90.0f,
    0.0f,
    252.0f
};


class Settings {
    public:
        Settings(const char *ns);
        void begin(SettingsData data);

        void getSettings(SettingsData *data);
        void setSettings(SettingsData *data);

    private:
        Preferences preferences;

        const char* ns;
};