

#include "text_widget_highlighter.hpp"

// local
#include "qt_str.hpp"

using namespace tool::ui;

CSharpHighlighter::CSharpHighlighter(QTextDocument *parent): QSyntaxHighlighter(parent) {

    HighlightingRule rule;

    // set formats
    keywordFormat.setForeground(QColor(82,156,214));
    keywordFormat.setFontWeight(QFont::Bold);

    digitFormat.setForeground(QColor(167,206,168));

    typeFormat.setForeground(QColor(82,156,214));
    typeFormat.setFontWeight(QFont::Bold);

    structInstancesFormat.setForeground(QColor(134,176,95));
    structInstancesFormat.setFontWeight(QFont::Bold);

    classesInstancesFormat.setForeground(QColor(63, 201, 176));
    classesInstancesFormat.setFontWeight(QFont::Bold);


    classFormat.setFontWeight(QFont::Bold);
    classFormat.setForeground(QColor(63, 201, 176));

    quotationFormat.setForeground(QColor(214, 157, 133));

    functionFormat.setFontItalic(true);
    functionFormat.setForeground(QColor(220,220,200));

    singleLineCommentFormat.setForeground(QColor(87, 166, 70));
    multiLineCommentFormat.setForeground(QColor(87, 166, 70));

    defaultFormat.setForeground(QColor(220,220,200));

    // class
    rule.pattern = QRegularExpression(QSL("\\b[A-Za-z]+\\b"));
    rule.format = classFormat;
    //    highlightingRules.append(rule);


    // digits
    rule.pattern = QRegularExpression(QSL("\\b[0-9]*[,.]?[0-9]*[f]?\\b"));
    rule.format = digitFormat;
    highlightingRules.append(rule);

    // function
    rule.pattern = QRegularExpression(QSL("\\b[A-Za-z0-9_]+(?=\\()"));
    rule.format = functionFormat;
    highlightingRules.append(rule);

    // keywords
    const QString keywordPatterns[] = {
        QSL("\\bdo\\b"), QSL("\\bsizeof\\b"), QSL("\\bthis\\b"),
        QSL("\\bref\\b"), QSL("\\binternal\\b"), QSL("\\bout\\b"),
        QSL("\\bvolatile\\b"), QSL("\\breturn\\b"),QSL("\\babstract\\b"),
        QSL("\\bas\\b"), QSL("\\bbase\\b"),QSL("\\btry\\b"),
        QSL("\\btry\\b"), QSL("\\bcatch\\b"), QSL("\\bdelegate\\b"),
        QSL("\\bchecked\\b"),QSL("\\bexplicit\\b"),
        QSL("\\bforeach\\b"), QSL("\\bgoto\\b"),
        QSL("\\bis\\b"),QSL("\\block\\b"), QSL("\\bevent\\b"),
        QSL("\\bdefault\\b"), QSL("\\bextern\\b"),
        QSL("\\bstackalloc\\b"), QSL("\\bin\\b"), QSL("\\bimplicit\\b"),
        QSL("\\bfinally\\b"),QSL("\\bfixed\\b"),
        QSL("\\bnull\\b"), QSL("\\boperator\\b"),
        QSL("\\bparams\\b"),QSL("\\breadonly\\b"),
        QSL("\\bnew\\b"), QSL("\\bcontinue\\b"),
        QSL("\\bsealed\\b"),QSL("\\bthrow\\b"), QSL("\\bunsafe\\b"),
        QSL("\\bunchecked\\b"),QSL("\\btypeof\\b"),
        QSL("\\bswitch\\b"), QSL("\\bcase\\b"),QSL("\\bbreak\\b"),
        QSL("\\bwhile\\b"), QSL("\\bfor\\b"),
        QSL("\\bif\\b"), QSL("\\belse\\b"),
        QSL("\\btrue\\b"), QSL("\\bfalse\\b"),
        QSL("\\bclass\\b"), QSL("\\bconst\\b"),
        QSL("\\bEnum\\b"), QSL("\\busing\\b"),
        QSL("\\bprivate\\b"), QSL("\\bprotected\\b"), QSL("\\bpublic\\b"),
        QSL("\\bstatic\\b"), QSL("\\bstruct\\b"),
        QSL("\\bvirtual\\b"), QSL("\\boverride\\b"), QSL("\\bnamespace\\b")
    };

    for (const QString &pattern : keywordPatterns) {
        rule.pattern = QRegularExpression(pattern);
        rule.format = keywordFormat;
        highlightingRules.append(rule);
    }

        // types
        const QString typesPatterns[] = {
            QSL("\\bvoid\\b"), QSL("\\bvar\\b"),
            // built-in values types
            QSL("\\bbool\\b"),
            QSL("\\bbyte\\b"), QSL("\\bsbyte\\b"),
            QSL("\\bchar\\b"),
            QSL("\\bdecimal\\b"),
            QSL("\\bdouble\\b"), QSL("\\bfloat\\b"),
            QSL("\\buint\\b"), QSL("\\bint\\b"),
            QSL("\\bulong\\b"), QSL("\\blong\\b"),
            QSL("\\bushort\\b"), QSL("\\bshort\\b"),
            // built-in references types
            QSL("\\bstring\\b"), QSL("\\bobject\\b"), QSL("\\bdynamic\\b"),
            };

    for (const QString &pattern : typesPatterns) {
        rule.pattern = QRegularExpression(pattern);
        rule.format = typeFormat;
        highlightingRules.append(rule);
    }

        // structs instances
        const QString structsInstancesPatterns[] = {
            QSL("\\Color\\b"),
            QSL("\\bVector2\\b"), QSL("\\bVector3\\b"), QSL("\\bQuaternion\\b"),
            };
    for (const QString &pattern : structsInstancesPatterns) {
        rule.pattern = QRegularExpression(pattern);
        rule.format = structInstancesFormat;
        highlightingRules.append(rule);
    }

        // classes instances
        const QString classesInstancesPatterns[] = {
            QSL("\\bType\\b"),
            QSL("\\bArrayList\\b"), QSL("\\bHashtable\\b"),
            QSL("\\bList\\b"), QSL("\\bSortedList\\b"),
            QSL("\\bQueue\\b"), QSL("\\bStack\\b"),
            QSL("\\bDictionary\\b"), QSL("\\bTuple\\b"),
            QSL("\\bGameObject\\b"),QSL("\\bTransform\\b"),
            QSL("\\bMesh\\b"),QSL("\\bAudioClip\\b"),
            QSL("\\bAudioSource\\b"),QSL("\\bTexture2D\\b"),
            // exvr
            //        QSL("\\ExComponent\\b"),QSL("\\bExConnector\\b"),
            };
    for (const QString &pattern : classesInstancesPatterns) {
        rule.pattern = QRegularExpression(pattern);
        rule.format = classesInstancesFormat;
        highlightingRules.append(rule);
    }

    // quotation
    rule.pattern = QRegularExpression(QSL("\".*\""));
    rule.format = quotationFormat;
    highlightingRules.append(rule);

    // single line comment
    rule.pattern = QRegularExpression(QSL("//[^\n]*"));
    rule.format = singleLineCommentFormat;
    highlightingRules.append(rule);

    // comments
    commentStartExpression = QRegularExpression(QSL("/\\*"));
    commentEndExpression = QRegularExpression(QSL("\\*/"));
}

void CSharpHighlighter::add_classes(const std::vector<QString> &classNames){
    for(const auto& name : classNames){
        highlightingRules.append(HighlightingRule{QRegularExpression(name), classesInstancesFormat});
    }
}

void CSharpHighlighter::highlightBlock(const QString &text){

    setFormat(0, text.length(), defaultFormat);

    for (const HighlightingRule &rule : qAsConst(highlightingRules)) {
        QRegularExpressionMatchIterator matchIterator = rule.pattern.globalMatch(text);

        if(!matchIterator.hasNext()){

            continue;
        }

        while (matchIterator.hasNext()) {
            QRegularExpressionMatch match = matchIterator.next();
            setFormat(match.capturedStart(), match.capturedLength(), rule.format);
        }
    }

    setCurrentBlockState(0);

    int startIndex = 0;
    if (previousBlockState() != 1){
        startIndex = text.indexOf(commentStartExpression);
    }

    while (startIndex >= 0) {
        QRegularExpressionMatch match = commentEndExpression.match(text, startIndex);
        int endIndex = match.capturedStart();
        int commentLength = 0;
        if (endIndex == -1) {
            setCurrentBlockState(1);
            commentLength = text.length() - startIndex;
        } else {
            commentLength = endIndex - startIndex
                            + match.capturedLength();
        }
        setFormat(startIndex, commentLength, multiLineCommentFormat);
        startIndex = text.indexOf(commentStartExpression, startIndex + commentLength);
    }
}
