#include "./settings.h"


Settings::Settings(const char* ns) {
    this->ns = ns;
}

void Settings::getSettings(SettingsData *data) {
    this->preferences.begin(this->ns, true);

    data->range = preferences.getFloat("range", 0.0f);
    data->offset = preferences.getFloat("offset", 0.0f);
    data->stepsPerDegree = preferences.getFloat("spd", 0.0f);

    this->preferences.end();
}

void Settings::setSettings(SettingsData *data) {
    this->preferences.begin(this->ns, false);

    this->preferences.putFloat("range", data->range);
    this->preferences.putFloat("offset", data->offset);
    this->preferences.putFloat("spd", data->stepsPerDegree);

    this->preferences.end();
}

void Settings::begin(SettingsData data) {
    this->preferences.begin(this->ns, false);

    if (!this->preferences.isKey("range")) this->preferences.putFloat("range", data.range);
    if (!this->preferences.isKey("offset")) this->preferences.putFloat("offset", data.offset);
    if (!this->preferences.isKey("spd")) this->preferences.putFloat("spd", data.stepsPerDegree);

    this->preferences.end();
}