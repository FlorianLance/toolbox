

#include "text_widget_highlighter.hpp"

// local
#include "qt_str.hpp"

using namespace tool::ui;

CSharpHighlighter::CSharpHighlighter(QTextDocument *parent, QVector<HighlightingRule> *highlightingRules): QSyntaxHighlighter(parent), highlightingRules(highlightingRules) {
}

void CSharpHighlighter::highlightBlock(const QString &text){

    setFormat(0, text.length(), CSharpHighlighting::defaultFormat);

    for (const HighlightingRule &rule : qAsConst(*highlightingRules)) {
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
        startIndex = text.indexOf(CSharpHighlighting::commentStartExpression);
    }

    while (startIndex >= 0) {
        QRegularExpressionMatch match = CSharpHighlighting::commentEndExpression.match(text, startIndex);
        int endIndex = match.capturedStart();
        int commentLength = 0;
        if (endIndex == -1) {
            setCurrentBlockState(1);
            commentLength = text.length() - startIndex;
        } else {
            commentLength = endIndex - startIndex
                            + match.capturedLength();
        }
        setFormat(startIndex, commentLength, CSharpHighlighting::multiLineCommentFormat);
        startIndex = text.indexOf(CSharpHighlighting::commentStartExpression, startIndex + commentLength);
    }
}
