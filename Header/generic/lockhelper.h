#ifndef LOCKHELPER_H
#define LOCKHELPER_H

#pragma once

#include <QtCore/private/qandroidextras_p.h>
#include <QJniObject>


class KeepAwakeHelper

{

public:

    KeepAwakeHelper();

    ~KeepAwakeHelper();

    void EnableKeepAwakeHelper();

    void DisableKeepAwakeHelper();

};

#endif // LOCKHELPER_H
