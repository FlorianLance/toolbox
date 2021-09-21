
/*******************************************************************************
** exvr-designer                                                              **
** No license (to be defined)                                                 **
** Copyright (c) [2018] [Florian Lance][EPFL-LNCO]                            **
********************************************************************************/

#pragma once

// Qt
#include <QObject>
#include <QString>

// local
#include "qt_str.hpp"

namespace tool {


class QtLogger : public QObject{

Q_OBJECT
public:

    enum class MessageType : int{
        normal, warning, error, unknow,
    };


    QtLogger();

    static QtLogger *get();

    static void set_html_ui_type_message_color(MessageType type, const QColor &col);
//    static void set_as_console_type_message(MessageType type);

    static void init(QString logDirectoryPath = "", QString logFileName = "default_log.html");

    static void message(const QString &message, bool triggersSignal = true, bool saveToFile = true);
    static void error(const QString &error, bool triggersSignal = true, bool saveToFile = true);
    static void warning(const QString &warning, bool triggersSignal = true, bool saveToFile = true);

//    static void unity_message(const QString &message, bool triggersSignal = true, bool saveToFile = true);
//    static void unity_error(const QString &error, bool triggersSignal = true, bool saveToFile = true);
//    static void unity_warning(const QString &warning, bool triggersSignal = true, bool saveToFile = true);
//    static void unity_unknow(const QString &unknow, bool triggersSignal = true, bool saveToFile = true);

    static void unity_message(QStringView message, bool triggersSignal = true, bool saveToFile = true);
    static void unity_error(QStringView error, bool triggersSignal = true, bool saveToFile = true);
    static void unity_warning(QStringView warning, bool triggersSignal = true, bool saveToFile = true);
    static void unity_unknow(QStringView unknow, bool triggersSignal = true, bool saveToFile = true);

    static void status(const QString &status, int ms = 0);
    static void progress(int state);

    static void clean();

private:

    static QString to_html_line(QtLogger::MessageType type, QStringView text, bool addTimestamp);
    static void insert_line_to_log_file(MessageType type, QStringView message);


signals:

    void message_signal(QString message);
    void error_signal(QString error);
    void warning_signal(QString warning);
    void status_signal(QString status, int ms);

    void unity_message_signal(QStringView message);
    void unity_error_signal(QStringView error);
    void unity_warning_signal(QStringView warning);
    void unity_unknow_signal(QStringView unknow);

//    void unity_message_signal(QStringView message);
//    void unity_error_signal(QStringView error);
//    void unity_warning_signal(QStringView warning);
//    void unity_unknow_signal(QStringView unknow);

    void progress_signal(int state);

private:

    struct Impl;
    std::unique_ptr<Impl> m_p = nullptr;
};
}

