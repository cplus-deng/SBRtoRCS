#pragma once
#ifndef _SILENCE_AMP_DEPRECATION_WARNINGS
#define _SILENCE_AMP_DEPRECATION_WARNINGS
#endif
#include <QMainWindow>
#include "ui_TypeWidget.h"
#include "TypeDef.hpp"
QT_BEGIN_NAMESPACE
namespace Ui { class TypeWidgetClass; };
QT_END_NAMESPACE

class TypeWidget : public QMainWindow
{
	Q_OBJECT

public:
	TypeWidget(QWidget *parent = nullptr);
	~TypeWidget();
public slots:
	void receiveEditTypeFromMainWin(QStringList data);
	void onCoatOkBtnClicked();
	void onCoatSaveBtnClicked();
	void onDieOkBtnClicked();
	void onDieSaveBtnClicked();
signals:
	void sendNewTypeToMainWin(QStringList data);
	void sendEditTypeToMainWin(QStringList data);
private:
	Ui::TypeWidgetClass *ui;
};
