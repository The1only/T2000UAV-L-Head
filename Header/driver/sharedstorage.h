#ifndef SHAREDSTORAGE_H
#define SHAREDSTORAGE_H

#include <QString>

class SharedStorage
{
public:
    static bool appendTextFile(
        const QString &directory,
        const QString &fileName,
        const QString &text);

    static bool writeTextFile(
        const QString &directory,
        const QString &fileName,
        const QString &text);
};

#endif // SHAREDSTORAGE_H