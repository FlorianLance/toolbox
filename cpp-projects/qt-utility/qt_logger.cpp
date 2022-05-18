
/*******************************************************************************
** Toolbox-qt-utility                                                         **
** MIT License                                                                **
** Copyright (c) [2018] [Florian Lance]                                       **
**                                                                            **
** Permission is hereby granted, free of charge, to any person obtaining a    **
** copy of this software and associated documentation files (the "Software"), **
** to deal in the Software without restriction, including without limitation  **
** the rights to use, copy, modify, merge, publish, distribute, sublicense,   **
** and/or sell copies of the Software, and to permit persons to whom the      **
** Software is furnished to do so, subject to the following conditions:       **
**                                                                            **
** The above copyright notice and this permission notice shall be included in **
** all copies or substantial portions of the Software.                        **
**                                                                            **
** THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR **
** IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,   **
** FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL    **
** THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER **
** LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING    **
** FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER        **
** DEALINGS IN THE SOFTWARE.                                                  **
**                                                                            **
********************************************************************************/

#include "qt_logger.hpp"

// std
#include <mutex>

// Qt
#include <QDir>
#include <QFile>
#include <QApplication>
#include <QDateTime>
#include <QHostInfo>
#include <QString>
#include <QColor>

// base
//#include "utility/logger.hpp"
#include "utility/time.hpp"

// qt-utility
#include "qt_str.hpp"

using namespace tool;


struct QtLogger::Impl{

    static inline bool displayConsole = false;
    static inline std::mutex locker;
    static inline std::unique_ptr<QtLogger> logger = nullptr;
    static inline std::unique_ptr<QFile> file = nullptr;
    static inline std::unique_ptr<QTextStream> out = nullptr;

    static inline std::chrono::nanoseconds epochStart;

    static const inline QString startHtmlBalise           = QSL("<p>");
    static const inline QString startTimestampHtmlBalise  = QSL("<p> [");
    static const inline QString midTimestampHtmlBalise    = QSL("] ");
    static const inline QString endHtmlBalise             = QSL("</font></p>\n");

    static inline QString htmlNormalBaliseColor     = QSL("<font color=\"DarkBlue\">");
    static inline QString htmlWarningBaliseColor    = QSL("<font color=\"Orange\">");
    static inline QString htmlErrorBaliseColor      = QSL("<font color=\"DarkRed\">");
    static inline QString htmlLogBaliseColor        = QSL("<font color=\"LightGrey\">");
    static inline QString htmlUnknowBaliseColor     = QSL("<font color=\"Black\">");

    Impl(){}
};

QtLogger::QtLogger() : m_p(std::make_unique<Impl>()){
}

QtLogger *QtLogger::get(){
    if(QtLogger::Impl::logger != nullptr){
        return QtLogger::Impl::logger.get();
    }
    return nullptr;
}

void QtLogger::set_html_ui_type_message_color(QtLogger::MessageType type, const QColor &col){

    const QString baliseStr = QSL("<font color=") % col.name() %QSL(">");
    switch (type) {
    case MessageType::normal:
        QtLogger::Impl::htmlNormalBaliseColor = baliseStr;
        break;
    case MessageType::warning:
        QtLogger::Impl::htmlWarningBaliseColor = baliseStr;
        break;
    case MessageType::error:
        QtLogger::Impl::htmlErrorBaliseColor = baliseStr;
        break;
    case MessageType::log:
        QtLogger::Impl::htmlLogBaliseColor = baliseStr;
        break;
    case MessageType::unknow:
        QtLogger::Impl::htmlUnknowBaliseColor = baliseStr;
        break;
    }
}

QString QtLogger::to_html_line(QtLogger::MessageType type, QStringView text, bool addTimestamp){

    QStringView colorCode;
    switch (type) {
    case QtLogger::MessageType::normal:
        colorCode = QtLogger::Impl::htmlNormalBaliseColor;
        break;
    case QtLogger::MessageType::warning:
        colorCode = QtLogger::Impl::htmlWarningBaliseColor;
        break;
    case QtLogger::MessageType::error:
        colorCode = QtLogger::Impl::htmlErrorBaliseColor;
        break;
    case QtLogger::MessageType::log:
        colorCode = QtLogger::Impl::htmlLogBaliseColor;
        break;
    case QtLogger::MessageType::unknow:
        colorCode = QtLogger::Impl::htmlUnknowBaliseColor;
        break;
    }

    if(!addTimestamp){
        return QString("%1%2%3%4").arg(QtLogger::Impl::startHtmlBalise, colorCode, text, QtLogger::Impl::endHtmlBalise);
    }else{
        return QString("%1%2%3%4%5%6").arg(
            QtLogger::Impl::startTimestampHtmlBalise,
            QString::number(std::chrono::duration_cast<std::chrono::milliseconds>(nanoseconds_since_epoch() - QtLogger::Impl::epochStart).count()),
            QtLogger::Impl::midTimestampHtmlBalise,
            colorCode,
            text,
            QtLogger::Impl::endHtmlBalise
        );
    }
}

void QtLogger::insert_line_to_log_file(QtLogger::MessageType type, QStringView message){

    std::unique_lock<std::mutex> lock(QtLogger::Impl::locker);
    if(QtLogger::Impl::out){
        (*QtLogger::Impl::out) << to_html_line(type, message, true) << Qt::flush;
    }
}

void QtLogger::init(QString logDirectoryPath, QString logFileName, bool displayConsole){

    QtLogger::Impl::displayConsole = displayConsole;

    if(logDirectoryPath.length() == 0 || logFileName.length() == 0){
        logDirectoryPath = QApplication::applicationDirPath() % QSL("/logs");
        logFileName      = QSL("default_") % QHostInfo::localHostName() % QSL(".html");
    }

    QString logDirPath       = logDirectoryPath;
    QString logName          = logFileName;
    QString absoluteFilePath = logDirPath % "/" % logName;

    if(!QFile::exists(logDirPath)){
        QDir logDir(logDirPath);
        if(!logDir.mkdir(logDirPath)){
            qWarning() << "[QtLogger-ERROR] Cannot create logging directory path.\n";
            return;
        }
    }

    std::unique_lock<std::mutex> lock(QtLogger::Impl::locker);
    if(QtLogger::Impl::logger == nullptr){

        if(QFile::exists(absoluteFilePath)){

            int id = absoluteFilePath.lastIndexOf('.');
            if(id != -1){
                QString leftPart  = absoluteFilePath.left(id);
                QString extension = absoluteFilePath.mid(id);
                QString previousFileName  = leftPart % QSL("_previous") % extension;

                bool copyPrevious = true;
                if(QFile::exists(previousFileName)){
                    if(!QFile::remove(previousFileName)){
                        qWarning() << "[QtLogger-ERROR] Cannot remove previous log file: " << previousFileName;
                        copyPrevious = false;
                    }
                }

                if(copyPrevious){
                    if(!QFile::copy(absoluteFilePath, previousFileName)){
                        qWarning() << "[QtLogger-ERROR] Cannot save last log file " << absoluteFilePath << " to " << previousFileName;
                    }
                }
            }else{
                qWarning() << "[QtLogger-ERROR] Invalid log file path, no extension: " << absoluteFilePath;
                return;
            }
        }

        // init log file        
        QtLogger::Impl::epochStart = nanoseconds_since_epoch();
        QtLogger::Impl::logger     = std::make_unique<QtLogger>();
        QtLogger::Impl::file       = std::make_unique<QFile>(absoluteFilePath);

        if(!QtLogger::Impl::file->open(QFile::WriteOnly | QFile::Text)){
            qWarning() << "[QtLogger-ERROR] Cannot write to log file: " << absoluteFilePath;
            return;
        }

        QtLogger::Impl::out = std::make_unique<QTextStream>(QtLogger::Impl::file.get());
    }
}

void QtLogger::clean(){

    std::unique_lock<std::mutex> lock(QtLogger::Impl::locker);
    if(QtLogger::Impl::file){
        QtLogger::Impl::file->close();
        QtLogger::Impl::file = nullptr;
        QtLogger::Impl::out  = nullptr;
    }
    QtLogger::Impl::logger = nullptr;
}

void QtLogger::message(const QString &message, bool triggersSignal, bool saveToFile){

    if(auto logger = QtLogger::get(); logger != nullptr){

        if(triggersSignal){
            emit QtLogger::Impl::logger->message_signal(
                to_html_line(MessageType::normal, message, false)
            );
            if(QtLogger::Impl::displayConsole){
                qDebug() << message;
            }
        }

        if(saveToFile){
            insert_line_to_log_file(MessageType::normal, message);
        }
    }
}

void QtLogger::error(const QString &error, bool triggersSignal, bool saveToFile){

    if(auto logger = QtLogger::get(); logger != nullptr){

        if(triggersSignal){
            emit QtLogger::Impl::logger->error_signal(
                to_html_line(MessageType::error, error, false)
            );
            if(QtLogger::Impl::displayConsole){
                qDebug() << error;
            }
        }

        if(saveToFile){
            insert_line_to_log_file(MessageType::error, error);
        }
    }
}

void QtLogger::warning(const QString &warning, bool triggersSignal, bool saveToFile){

    if(auto logger = QtLogger::get(); logger != nullptr){

        if(triggersSignal){
            emit QtLogger::Impl::logger->warning_signal(
                to_html_line(MessageType::warning, warning, false)
            );
            if(QtLogger::Impl::displayConsole){
                qWarning() << warning;
            }
        }

        if(saveToFile){
            insert_line_to_log_file(MessageType::warning, warning);
        }
    }
}

void QtLogger::log(const QString &log, bool triggersSignal, bool saveToFile){

    if(auto logger = QtLogger::get(); logger != nullptr){

        if(triggersSignal){
            emit QtLogger::Impl::logger->log_signal(
                to_html_line(MessageType::log, log, false)
            );
        }

        if(saveToFile){
            insert_line_to_log_file(MessageType::log, log);
        }
    }
}

void QtLogger::status(const QString &status, int ms){
    if(auto logger = QtLogger::get(); logger != nullptr){
        emit QtLogger::Impl::logger->status_signal(status, ms);
    }
}

void QtLogger::progress(int state){
    if(auto logger = QtLogger::get(); logger != nullptr){
        emit QtLogger::Impl::logger->progress_signal(state);
    }
}

void QtLogger::unity_message(QStringView message, bool triggersSignal, bool saveToFile){

    if(auto logger = QtLogger::get(); logger != nullptr){

        if(triggersSignal){
            int idRichText = message.indexOf(QSL("|R|"));
            if(idRichText != -1){
                emit QtLogger::Impl::logger->unity_message_signal(message.chopped(idRichText));
            }else{
                emit QtLogger::Impl::logger->unity_message_signal(message);
            }
        }

        if(saveToFile){
            insert_line_to_log_file(MessageType::normal, message);
        }
    }
}

void QtLogger::unity_error(QStringView error, bool triggersSignal, bool saveToFile){

    if(auto logger = QtLogger::get(); logger != nullptr){

        if(triggersSignal){
//            int idRichText = error.indexOf(QSL("<font color="));
//            if(idRichText != -1){
//                emit QtLogger::Impl::logger->unity_error_signal(error.chopped(idRichText));
//            }else{
                emit QtLogger::Impl::logger->unity_error_signal(error);
//            }
        }

        if(saveToFile){
            insert_line_to_log_file(MessageType::error, error);
        }
    }
}

void QtLogger::unity_warning(QStringView warning, bool triggersSignal, bool saveToFile){

    if(auto logger = QtLogger::get(); logger != nullptr){

        if(triggersSignal){
//            int idRichText = warning.indexOf(QSL("<font color="));
//            if(idRichText != -1){
//                emit QtLogger::Impl::logger->unity_warning_signal(warning.chopped(idRichText));
//            }else{
                emit QtLogger::Impl::logger->unity_warning_signal(warning);
//            }
        }

        if(saveToFile){
            insert_line_to_log_file(MessageType::warning, warning);
        }
    }
}

void QtLogger::unity_unknow(QStringView unknow, bool triggersSignal, bool saveToFile){

    if(auto logger = QtLogger::get(); logger != nullptr){

        if(triggersSignal){
//            int idRichText = unknow.indexOf(QSL("<font color="));
//            if(idRichText != -1){
//                emit QtLogger::Impl::logger->unity_unknow_signal(unknow.chopped(idRichText));
//            }else{
                emit QtLogger::Impl::logger->unity_unknow_signal(unknow);
//            }
        }

        if(saveToFile){
            insert_line_to_log_file(MessageType::unknow, unknow);
        }
    }
}

// #include "moc_qt_logger.cpp"
