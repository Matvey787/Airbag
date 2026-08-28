#include <QCoreApplication>
#include <QDir>
#include <QString>

inline QString firmwareParamsFilePath()
{
    return QDir(QCoreApplication::applicationDirPath())
        .absoluteFilePath(QStringLiteral("@FIRMWARE_PARAMS_REL_PATH@"));
}

inline QString mapDirPath()
{
    return QDir(QCoreApplication::applicationDirPath())
        .absoluteFilePath(QStringLiteral("@MAP_DIR_REL_PATH@"));
}