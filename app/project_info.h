#include <QCoreApplication>
#include <QDir>
#include <QString>

inline QString firmwareParamsFilePath()
{
    return QDir(QCoreApplication::applicationDirPath())
        .absoluteFilePath(QStringLiteral("../../../firmwares/esp32_firmware/data/firmware_vars.json"));
}

inline QString mapDirPath()
{
    return QDir(QCoreApplication::applicationDirPath())
        .absoluteFilePath(QStringLiteral("../../map/"));
}
