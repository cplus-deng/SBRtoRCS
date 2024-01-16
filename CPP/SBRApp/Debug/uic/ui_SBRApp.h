/********************************************************************************
** Form generated from reading UI file 'SBRApp.ui'
**
** Created by: Qt User Interface Compiler version 5.12.12
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_SBRAPP_H
#define UI_SBRAPP_H

#include <QtCore/QVariant>
#include <QtGui/QIcon>
#include <QtWidgets/QApplication>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QFrame>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QTreeView>
#include <QtWidgets/QWidget>
#include "qchartview.h"

QT_BEGIN_NAMESPACE

class Ui_SBRAppClass
{
public:
    QWidget *centralWidget;
    QLabel *label_14;
    QFrame *line_3;
    QLabel *label_15;
    QLineEdit *thetaIn;
    QLabel *label_16;
    QLabel *label_17;
    QLineEdit *phiIn;
    QLabel *label_18;
    QLabel *label_19;
    QLineEdit *inWavePol;
    QLabel *label_20;
    QLineEdit *obsPol;
    QLabel *label_21;
    QFrame *line_4;
    QLineEdit *rayPerLambda;
    QTextEdit *runMsg;
    QCheckBox *enterInTrCheck;
    QPushButton *runButton;
    QPushButton *resetButton;
    QPushButton *cancelButton;
    QPushButton *confirmButton;
    QWidget *meshWidget;
    QTextEdit *loadMsg;
    QChartView *freqLine;
    QLabel *label_28;
    QLabel *label_29;
    QLabel *label_26;
    QLabel *label_30;
    QLabel *label_31;
    QFrame *line_5;
    QTabWidget *tabWidget;
    QWidget *tab;
    QLabel *label_32;
    QPushButton *pecImport;
    QWidget *tab_2;
    QLabel *label_33;
    QPushButton *coatImport;
    QLabel *label_35;
    QLabel *label_36;
    QLineEdit *coatMuR;
    QLabel *label_37;
    QLineEdit *coatMuI;
    QLabel *label_38;
    QLabel *label_39;
    QLabel *label_40;
    QLineEdit *coatEpsI;
    QLineEdit *coatEpsR;
    QLabel *label_47;
    QLineEdit *coatThickIn;
    QLabel *label_48;
    QWidget *tab_3;
    QLabel *label_34;
    QPushButton *dieleImport;
    QLabel *label_41;
    QLabel *label_42;
    QLabel *label_43;
    QLabel *label_44;
    QLineEdit *dieEpsI;
    QLabel *label_45;
    QLineEdit *dieMuI;
    QLabel *label_46;
    QLineEdit *dieMuR;
    QLineEdit *dieEpsR;
    QLabel *label_49;
    QPushButton *addPolX;
    QPushButton *rmPolX;
    QChartView *freqLineX;
    QFrame *line;
    QLabel *label;
    QLabel *label_2;
    QLabel *chartHead1;
    QLabel *chartHead2;
    QLabel *label_27;
    QLineEdit *freqHigh;
    QLabel *label_23;
    QLabel *label_22;
    QLabel *label_24;
    QLabel *label_25;
    QLineEdit *freqLow;
    QWidget *layoutWidget;
    QHBoxLayout *obsPolXLayout;
    QLineEdit *obsPolX;
    QLabel *obsPolXDeg;
    QTreeView *treeView;
    QPushButton *importButton;
    QPushButton *importCatButton;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *SBRAppClass)
    {
        if (SBRAppClass->objectName().isEmpty())
            SBRAppClass->setObjectName(QString::fromUtf8("SBRAppClass"));
        SBRAppClass->resize(1920, 1080);
        SBRAppClass->setCursor(QCursor(Qt::ArrowCursor));
        SBRAppClass->setFocusPolicy(Qt::StrongFocus);
        QIcon icon;
        icon.addFile(QString::fromUtf8(":/SBRApp/icon/App.png"), QSize(), QIcon::Normal, QIcon::Off);
        SBRAppClass->setWindowIcon(icon);
        centralWidget = new QWidget(SBRAppClass);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        label_14 = new QLabel(centralWidget);
        label_14->setObjectName(QString::fromUtf8("label_14"));
        label_14->setGeometry(QRect(84, 382, 271, 41));
        QFont font;
        font.setFamily(QString::fromUtf8("Microsoft JhengHei UI"));
        font.setPointSize(16);
        label_14->setFont(font);
        line_3 = new QFrame(centralWidget);
        line_3->setObjectName(QString::fromUtf8("line_3"));
        line_3->setGeometry(QRect(34, 412, 311, 16));
        line_3->setFrameShape(QFrame::HLine);
        line_3->setFrameShadow(QFrame::Sunken);
        label_15 = new QLabel(centralWidget);
        label_15->setObjectName(QString::fromUtf8("label_15"));
        label_15->setGeometry(QRect(34, 432, 51, 28));
        QFont font1;
        font1.setFamily(QString::fromUtf8("Times New Roman"));
        font1.setPointSize(16);
        label_15->setFont(font1);
        thetaIn = new QLineEdit(centralWidget);
        thetaIn->setObjectName(QString::fromUtf8("thetaIn"));
        thetaIn->setGeometry(QRect(84, 432, 71, 28));
        QFont font2;
        font2.setPointSize(16);
        thetaIn->setFont(font2);
        label_16 = new QLabel(centralWidget);
        label_16->setObjectName(QString::fromUtf8("label_16"));
        label_16->setGeometry(QRect(164, 432, 16, 28));
        QFont font3;
        font3.setFamily(QString::fromUtf8("Arial"));
        font3.setPointSize(16);
        label_16->setFont(font3);
        label_17 = new QLabel(centralWidget);
        label_17->setObjectName(QString::fromUtf8("label_17"));
        label_17->setGeometry(QRect(194, 432, 41, 28));
        label_17->setFont(font1);
        phiIn = new QLineEdit(centralWidget);
        phiIn->setObjectName(QString::fromUtf8("phiIn"));
        phiIn->setGeometry(QRect(234, 432, 71, 28));
        phiIn->setFont(font2);
        label_18 = new QLabel(centralWidget);
        label_18->setObjectName(QString::fromUtf8("label_18"));
        label_18->setGeometry(QRect(310, 432, 16, 28));
        label_18->setFont(font3);
        label_19 = new QLabel(centralWidget);
        label_19->setObjectName(QString::fromUtf8("label_19"));
        label_19->setGeometry(QRect(34, 472, 61, 28));
        label_19->setFont(font1);
        inWavePol = new QLineEdit(centralWidget);
        inWavePol->setObjectName(QString::fromUtf8("inWavePol"));
        inWavePol->setGeometry(QRect(84, 470, 71, 28));
        inWavePol->setFont(font2);
        label_20 = new QLabel(centralWidget);
        label_20->setObjectName(QString::fromUtf8("label_20"));
        label_20->setGeometry(QRect(84, 682, 251, 28));
        label_20->setFont(font);
        obsPol = new QLineEdit(centralWidget);
        obsPol->setObjectName(QString::fromUtf8("obsPol"));
        obsPol->setGeometry(QRect(90, 730, 71, 28));
        obsPol->setFont(font2);
        label_21 = new QLabel(centralWidget);
        label_21->setObjectName(QString::fromUtf8("label_21"));
        label_21->setGeometry(QRect(84, 532, 271, 41));
        label_21->setFont(font);
        line_4 = new QFrame(centralWidget);
        line_4->setObjectName(QString::fromUtf8("line_4"));
        line_4->setGeometry(QRect(34, 562, 311, 16));
        line_4->setFrameShape(QFrame::HLine);
        line_4->setFrameShadow(QFrame::Sunken);
        rayPerLambda = new QLineEdit(centralWidget);
        rayPerLambda->setObjectName(QString::fromUtf8("rayPerLambda"));
        rayPerLambda->setGeometry(QRect(300, 470, 71, 28));
        rayPerLambda->setFont(font2);
        runMsg = new QTextEdit(centralWidget);
        runMsg->setObjectName(QString::fromUtf8("runMsg"));
        runMsg->setGeometry(QRect(30, 840, 491, 171));
        QFont font4;
        font4.setFamily(QString::fromUtf8("Times New Roman"));
        font4.setPointSize(12);
        runMsg->setFont(font4);
        runMsg->setReadOnly(true);
        runMsg->setAcceptRichText(false);
        enterInTrCheck = new QCheckBox(centralWidget);
        enterInTrCheck->setObjectName(QString::fromUtf8("enterInTrCheck"));
        enterInTrCheck->setGeometry(QRect(30, 330, 171, 31));
        QFont font5;
        font5.setFamily(QString::fromUtf8("Times New Roman"));
        font5.setPointSize(14);
        enterInTrCheck->setFont(font5);
        runButton = new QPushButton(centralWidget);
        runButton->setObjectName(QString::fromUtf8("runButton"));
        runButton->setGeometry(QRect(560, 950, 50, 50));
        QFont font6;
        font6.setFamily(QString::fromUtf8("Microsoft Sans Serif"));
        font6.setPointSize(18);
        runButton->setFont(font6);
        runButton->setCursor(QCursor(Qt::PointingHandCursor));
        runButton->setFocusPolicy(Qt::NoFocus);
        runButton->setStyleSheet(QString::fromUtf8("background-color:transparent;\n"
"border-color: none;"));
        QIcon icon1;
        icon1.addFile(QString::fromUtf8(":/SBRApp/icon/\350\277\220\350\241\214\344\270\255.png"), QSize(), QIcon::Normal, QIcon::Off);
        icon1.addFile(QString::fromUtf8(":/SBRApp/icon/\346\232\202\345\201\234.png"), QSize(), QIcon::Normal, QIcon::On);
        icon1.addFile(QString::fromUtf8(":/SBRApp/icon/\346\232\202\345\201\234.png"), QSize(), QIcon::Disabled, QIcon::On);
        runButton->setIcon(icon1);
        runButton->setIconSize(QSize(40, 40));
        runButton->setCheckable(true);
        runButton->setChecked(false);
        resetButton = new QPushButton(centralWidget);
        resetButton->setObjectName(QString::fromUtf8("resetButton"));
        resetButton->setGeometry(QRect(640, 950, 50, 50));
        resetButton->setFont(font6);
        resetButton->setCursor(QCursor(Qt::PointingHandCursor));
        resetButton->setStyleSheet(QString::fromUtf8("background-color:transparent;\n"
"border-color: none;"));
        QIcon icon2;
        icon2.addFile(QString::fromUtf8(":/SBRApp/icon/\351\207\215\347\275\256.png"), QSize(), QIcon::Normal, QIcon::Off);
        icon2.addFile(QString::fromUtf8(":/SBRApp/icon/\347\241\256\350\256\244.png"), QSize(), QIcon::Normal, QIcon::On);
        resetButton->setIcon(icon2);
        resetButton->setIconSize(QSize(40, 40));
        resetButton->setCheckable(true);
        resetButton->setAutoRepeat(false);
        cancelButton = new QPushButton(centralWidget);
        cancelButton->setObjectName(QString::fromUtf8("cancelButton"));
        cancelButton->setGeometry(QRect(720, 950, 101, 71));
        cancelButton->setFont(font6);
        cancelButton->setStyleSheet(QString::fromUtf8("QPushButton::hover {\n"
"    border-style: solid;\n"
"    border-width: 0px;\n"
"    border-radius: 0px;\n"
"}\n"
"QPushButton::focus{outline: none;}"));
        confirmButton = new QPushButton(centralWidget);
        confirmButton->setObjectName(QString::fromUtf8("confirmButton"));
        confirmButton->setGeometry(QRect(30, 800, 181, 31));
        QFont font7;
        font7.setFamily(QString::fromUtf8("Microsoft YaHei UI"));
        font7.setPointSize(11);
        confirmButton->setFont(font7);
        QIcon icon3;
        icon3.addFile(QString::fromUtf8(":/SBRApp/icon/parameter\347\241\256\350\256\244.png"), QSize(), QIcon::Normal, QIcon::Off);
        confirmButton->setIcon(icon3);
        meshWidget = new QWidget(centralWidget);
        meshWidget->setObjectName(QString::fromUtf8("meshWidget"));
        meshWidget->setGeometry(QRect(540, 10, 900, 900));
        meshWidget->setStyleSheet(QString::fromUtf8("background-color: rgb(245, 255, 238);"));
        loadMsg = new QTextEdit(centralWidget);
        loadMsg->setObjectName(QString::fromUtf8("loadMsg"));
        loadMsg->setGeometry(QRect(540, 920, 901, 31));
        loadMsg->setFont(font4);
        loadMsg->setStyleSheet(QString::fromUtf8("background-color:transparent;\n"
"border-color: none;"));
        freqLine = new QChartView(centralWidget);
        freqLine->setObjectName(QString::fromUtf8("freqLine"));
        freqLine->setGeometry(QRect(1460, 90, 451, 381));
        freqLine->setCursor(QCursor(Qt::ArrowCursor));
        freqLine->setStyleSheet(QString::fromUtf8("background-color:transparent;"));
        label_28 = new QLabel(centralWidget);
        label_28->setObjectName(QString::fromUtf8("label_28"));
        label_28->setGeometry(QRect(34, 522, 50, 50));
        label_28->setPixmap(QPixmap(QString::fromUtf8(":/SBRApp/icon/\346\227\266\345\237\237\346\263\242\345\275\242.png")));
        label_28->setScaledContents(true);
        label_29 = new QLabel(centralWidget);
        label_29->setObjectName(QString::fromUtf8("label_29"));
        label_29->setGeometry(QRect(34, 372, 50, 50));
        label_29->setPixmap(QPixmap(QString::fromUtf8(":/SBRApp/icon/\345\205\211\345\220\210\350\276\220\345\260\204.png")));
        label_29->setScaledContents(true);
        label_26 = new QLabel(centralWidget);
        label_26->setObjectName(QString::fromUtf8("label_26"));
        label_26->setGeometry(QRect(164, 472, 16, 28));
        label_26->setFont(font3);
        label_30 = new QLabel(centralWidget);
        label_30->setObjectName(QString::fromUtf8("label_30"));
        label_30->setGeometry(QRect(34, 662, 50, 50));
        label_30->setPixmap(QPixmap(QString::fromUtf8(":/SBRApp/icon/\346\226\271\345\220\221.png")));
        label_30->setScaledContents(true);
        label_31 = new QLabel(centralWidget);
        label_31->setObjectName(QString::fromUtf8("label_31"));
        label_31->setGeometry(QRect(160, 730, 16, 28));
        label_31->setFont(font3);
        line_5 = new QFrame(centralWidget);
        line_5->setObjectName(QString::fromUtf8("line_5"));
        line_5->setGeometry(QRect(34, 712, 311, 16));
        line_5->setFrameShape(QFrame::HLine);
        line_5->setFrameShadow(QFrame::Sunken);
        tabWidget = new QTabWidget(centralWidget);
        tabWidget->setObjectName(QString::fromUtf8("tabWidget"));
        tabWidget->setGeometry(QRect(0, 0, 101, 71));
        tabWidget->setStyleSheet(QString::fromUtf8("font: 14pt \"Microsoft JhengHei UI\";"));
        tab = new QWidget();
        tab->setObjectName(QString::fromUtf8("tab"));
        label_32 = new QLabel(tab);
        label_32->setObjectName(QString::fromUtf8("label_32"));
        label_32->setGeometry(QRect(10, 0, 401, 221));
        QFont font8;
        font8.setFamily(QString::fromUtf8("Times New Roman"));
        font8.setPointSize(12);
        font8.setBold(false);
        font8.setItalic(false);
        font8.setWeight(50);
        label_32->setFont(font8);
        label_32->setStyleSheet(QString::fromUtf8("font: 12pt \"Times New Roman\";"));
        label_32->setWordWrap(true);
        pecImport = new QPushButton(tab);
        pecImport->setObjectName(QString::fromUtf8("pecImport"));
        pecImport->setGeometry(QRect(260, 230, 131, 41));
        QFont font9;
        font9.setFamily(QString::fromUtf8("Microsoft YaHei UI"));
        font9.setPointSize(11);
        font9.setBold(false);
        font9.setItalic(false);
        font9.setWeight(50);
        pecImport->setFont(font9);
        pecImport->setCursor(QCursor(Qt::PointingHandCursor));
        pecImport->setStyleSheet(QString::fromUtf8("font: 11pt \"Microsoft YaHei UI\";"));
        QIcon icon4;
        icon4.addFile(QString::fromUtf8(":/SBRApp/icon/\351\200\211\346\213\251\346\226\207\344\273\266.png"), QSize(), QIcon::Normal, QIcon::Off);
        pecImport->setIcon(icon4);
        tabWidget->addTab(tab, QString());
        tab_2 = new QWidget();
        tab_2->setObjectName(QString::fromUtf8("tab_2"));
        label_33 = new QLabel(tab_2);
        label_33->setObjectName(QString::fromUtf8("label_33"));
        label_33->setGeometry(QRect(10, 0, 391, 161));
        label_33->setFont(font8);
        label_33->setStyleSheet(QString::fromUtf8("font: 12pt \"Times New Roman\";"));
        label_33->setWordWrap(true);
        coatImport = new QPushButton(tab_2);
        coatImport->setObjectName(QString::fromUtf8("coatImport"));
        coatImport->setGeometry(QRect(260, 230, 131, 41));
        coatImport->setFont(font9);
        coatImport->setCursor(QCursor(Qt::PointingHandCursor));
        coatImport->setStyleSheet(QString::fromUtf8("font: 11pt \"Microsoft YaHei UI\";"));
        coatImport->setIcon(icon4);
        label_35 = new QLabel(tab_2);
        label_35->setObjectName(QString::fromUtf8("label_35"));
        label_35->setGeometry(QRect(20, 160, 121, 31));
        QFont font10;
        font10.setFamily(QString::fromUtf8("Times New Roman"));
        font10.setPointSize(13);
        font10.setBold(false);
        font10.setItalic(false);
        font10.setWeight(50);
        label_35->setFont(font10);
        label_35->setStyleSheet(QString::fromUtf8("font: 13pt \"Times New Roman\";"));
        label_36 = new QLabel(tab_2);
        label_36->setObjectName(QString::fromUtf8("label_36"));
        label_36->setGeometry(QRect(20, 190, 121, 31));
        label_36->setFont(font10);
        label_36->setStyleSheet(QString::fromUtf8("font: 13pt \"Times New Roman\";"));
        coatMuR = new QLineEdit(tab_2);
        coatMuR->setObjectName(QString::fromUtf8("coatMuR"));
        coatMuR->setGeometry(QRect(140, 160, 51, 20));
        coatMuR->setStyleSheet(QString::fromUtf8("background-color:transparent;\n"
"border-color: transparent;"));
        coatMuR->setClearButtonEnabled(false);
        label_37 = new QLabel(tab_2);
        label_37->setObjectName(QString::fromUtf8("label_37"));
        label_37->setGeometry(QRect(190, 160, 16, 16));
        coatMuI = new QLineEdit(tab_2);
        coatMuI->setObjectName(QString::fromUtf8("coatMuI"));
        coatMuI->setGeometry(QRect(210, 160, 51, 20));
        coatMuI->setStyleSheet(QString::fromUtf8("background-color:transparent;\n"
"border-color: transparent;"));
        coatMuI->setClearButtonEnabled(false);
        label_38 = new QLabel(tab_2);
        label_38->setObjectName(QString::fromUtf8("label_38"));
        label_38->setGeometry(QRect(260, 160, 16, 16));
        label_39 = new QLabel(tab_2);
        label_39->setObjectName(QString::fromUtf8("label_39"));
        label_39->setGeometry(QRect(260, 190, 16, 16));
        label_40 = new QLabel(tab_2);
        label_40->setObjectName(QString::fromUtf8("label_40"));
        label_40->setGeometry(QRect(190, 190, 16, 16));
        coatEpsI = new QLineEdit(tab_2);
        coatEpsI->setObjectName(QString::fromUtf8("coatEpsI"));
        coatEpsI->setGeometry(QRect(210, 190, 51, 20));
        coatEpsI->setStyleSheet(QString::fromUtf8("background-color:transparent;\n"
"border-color: transparent;"));
        coatEpsI->setClearButtonEnabled(false);
        coatEpsR = new QLineEdit(tab_2);
        coatEpsR->setObjectName(QString::fromUtf8("coatEpsR"));
        coatEpsR->setGeometry(QRect(140, 190, 51, 20));
        coatEpsR->setStyleSheet(QString::fromUtf8("background-color:transparent;\n"
"border-color: transparent;"));
        coatEpsR->setClearButtonEnabled(false);
        label_47 = new QLabel(tab_2);
        label_47->setObjectName(QString::fromUtf8("label_47"));
        label_47->setGeometry(QRect(290, 160, 21, 31));
        label_47->setFont(font10);
        label_47->setStyleSheet(QString::fromUtf8("font: 13pt \"Times New Roman\";"));
        coatThickIn = new QLineEdit(tab_2);
        coatThickIn->setObjectName(QString::fromUtf8("coatThickIn"));
        coatThickIn->setGeometry(QRect(310, 160, 51, 20));
        coatThickIn->setStyleSheet(QString::fromUtf8("background-color:transparent;\n"
"border-color: transparent;"));
        coatThickIn->setClearButtonEnabled(false);
        label_48 = new QLabel(tab_2);
        label_48->setObjectName(QString::fromUtf8("label_48"));
        label_48->setGeometry(QRect(360, 160, 21, 16));
        tabWidget->addTab(tab_2, QString());
        tab_3 = new QWidget();
        tab_3->setObjectName(QString::fromUtf8("tab_3"));
        label_34 = new QLabel(tab_3);
        label_34->setObjectName(QString::fromUtf8("label_34"));
        label_34->setGeometry(QRect(10, 0, 391, 151));
        label_34->setFont(font8);
        label_34->setStyleSheet(QString::fromUtf8("font: 12pt \"Times New Roman\";"));
        label_34->setWordWrap(true);
        dieleImport = new QPushButton(tab_3);
        dieleImport->setObjectName(QString::fromUtf8("dieleImport"));
        dieleImport->setGeometry(QRect(260, 230, 131, 41));
        dieleImport->setFont(font9);
        dieleImport->setCursor(QCursor(Qt::PointingHandCursor));
        dieleImport->setStyleSheet(QString::fromUtf8("font: 11pt \"Microsoft YaHei UI\";"));
        dieleImport->setIcon(icon4);
        label_41 = new QLabel(tab_3);
        label_41->setObjectName(QString::fromUtf8("label_41"));
        label_41->setGeometry(QRect(260, 170, 16, 16));
        label_42 = new QLabel(tab_3);
        label_42->setObjectName(QString::fromUtf8("label_42"));
        label_42->setGeometry(QRect(190, 170, 16, 16));
        label_43 = new QLabel(tab_3);
        label_43->setObjectName(QString::fromUtf8("label_43"));
        label_43->setGeometry(QRect(190, 200, 16, 16));
        label_44 = new QLabel(tab_3);
        label_44->setObjectName(QString::fromUtf8("label_44"));
        label_44->setGeometry(QRect(20, 170, 121, 31));
        label_44->setFont(font10);
        label_44->setStyleSheet(QString::fromUtf8("font: 13pt \"Times New Roman\";"));
        dieEpsI = new QLineEdit(tab_3);
        dieEpsI->setObjectName(QString::fromUtf8("dieEpsI"));
        dieEpsI->setGeometry(QRect(210, 200, 51, 20));
        dieEpsI->setStyleSheet(QString::fromUtf8("background-color:transparent;\n"
"border-color: transparent;"));
        dieEpsI->setClearButtonEnabled(false);
        label_45 = new QLabel(tab_3);
        label_45->setObjectName(QString::fromUtf8("label_45"));
        label_45->setGeometry(QRect(260, 200, 16, 16));
        dieMuI = new QLineEdit(tab_3);
        dieMuI->setObjectName(QString::fromUtf8("dieMuI"));
        dieMuI->setGeometry(QRect(210, 170, 51, 20));
        dieMuI->setStyleSheet(QString::fromUtf8("background-color:transparent;\n"
"border-color: transparent;"));
        dieMuI->setClearButtonEnabled(false);
        label_46 = new QLabel(tab_3);
        label_46->setObjectName(QString::fromUtf8("label_46"));
        label_46->setGeometry(QRect(20, 200, 121, 31));
        label_46->setFont(font10);
        label_46->setStyleSheet(QString::fromUtf8("font: 13pt \"Times New Roman\";"));
        dieMuR = new QLineEdit(tab_3);
        dieMuR->setObjectName(QString::fromUtf8("dieMuR"));
        dieMuR->setGeometry(QRect(140, 170, 51, 20));
        dieMuR->setStyleSheet(QString::fromUtf8("background-color:transparent;\n"
"border-color: transparent;"));
        dieMuR->setClearButtonEnabled(false);
        dieEpsR = new QLineEdit(tab_3);
        dieEpsR->setObjectName(QString::fromUtf8("dieEpsR"));
        dieEpsR->setGeometry(QRect(140, 200, 51, 20));
        dieEpsR->setStyleSheet(QString::fromUtf8("background-color:transparent;\n"
"border-color: transparent;"));
        dieEpsR->setClearButtonEnabled(false);
        tabWidget->addTab(tab_3, QString());
        label_49 = new QLabel(centralWidget);
        label_49->setObjectName(QString::fromUtf8("label_49"));
        label_49->setGeometry(QRect(40, 730, 61, 28));
        label_49->setFont(font1);
        addPolX = new QPushButton(centralWidget);
        addPolX->setObjectName(QString::fromUtf8("addPolX"));
        addPolX->setGeometry(QRect(190, 730, 28, 28));
        addPolX->setCursor(QCursor(Qt::PointingHandCursor));
        QIcon icon5;
        icon5.addFile(QString::fromUtf8(":/SBRApp/icon/\346\267\273\345\212\240.png"), QSize(), QIcon::Normal, QIcon::Off);
        addPolX->setIcon(icon5);
        addPolX->setIconSize(QSize(28, 28));
        rmPolX = new QPushButton(centralWidget);
        rmPolX->setObjectName(QString::fromUtf8("rmPolX"));
        rmPolX->setGeometry(QRect(270, 730, 28, 28));
        rmPolX->setCursor(QCursor(Qt::PointingHandCursor));
        QIcon icon6;
        icon6.addFile(QString::fromUtf8(":/SBRApp/icon/\345\207\217-\346\226\271\346\241\206.png"), QSize(), QIcon::Normal, QIcon::Off);
        rmPolX->setIcon(icon6);
        rmPolX->setIconSize(QSize(28, 28));
        freqLineX = new QChartView(centralWidget);
        freqLineX->setObjectName(QString::fromUtf8("freqLineX"));
        freqLineX->setGeometry(QRect(1460, 510, 451, 381));
        freqLineX->setStyleSheet(QString::fromUtf8("background-color:transparent;"));
        line = new QFrame(centralWidget);
        line->setObjectName(QString::fromUtf8("line"));
        line->setGeometry(QRect(1440, 20, 20, 871));
        line->setFrameShape(QFrame::VLine);
        line->setFrameShadow(QFrame::Sunken);
        label = new QLabel(centralWidget);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(1470, 20, 25, 25));
        label->setPixmap(QPixmap(QString::fromUtf8(":/SBRApp/icon/\346\233\262\347\272\277.png")));
        label->setScaledContents(true);
        label_2 = new QLabel(centralWidget);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setGeometry(QRect(1500, 21, 54, 21));
        QFont font11;
        font11.setFamily(QString::fromUtf8("Microsoft JhengHei UI"));
        font11.setPointSize(14);
        label_2->setFont(font11);
        chartHead1 = new QLabel(centralWidget);
        chartHead1->setObjectName(QString::fromUtf8("chartHead1"));
        chartHead1->setGeometry(QRect(1460, 60, 141, 31));
        chartHead1->setFont(font4);
        chartHead2 = new QLabel(centralWidget);
        chartHead2->setObjectName(QString::fromUtf8("chartHead2"));
        chartHead2->setGeometry(QRect(1460, 480, 141, 31));
        chartHead2->setFont(font4);
        label_27 = new QLabel(centralWidget);
        label_27->setObjectName(QString::fromUtf8("label_27"));
        label_27->setGeometry(QRect(194, 472, 119, 24));
        label_27->setFont(font1);
        freqHigh = new QLineEdit(centralWidget);
        freqHigh->setObjectName(QString::fromUtf8("freqHigh"));
        freqHigh->setGeometry(QRect(140, 574, 78, 28));
        freqHigh->setFont(font2);
        label_23 = new QLabel(centralWidget);
        label_23->setObjectName(QString::fromUtf8("label_23"));
        label_23->setGeometry(QRect(224, 574, 39, 24));
        label_23->setFont(font1);
        label_22 = new QLabel(centralWidget);
        label_22->setObjectName(QString::fromUtf8("label_22"));
        label_22->setGeometry(QRect(36, 574, 98, 24));
        label_22->setFont(font1);
        label_24 = new QLabel(centralWidget);
        label_24->setObjectName(QString::fromUtf8("label_24"));
        label_24->setGeometry(QRect(36, 610, 98, 24));
        label_24->setFont(font1);
        label_25 = new QLabel(centralWidget);
        label_25->setObjectName(QString::fromUtf8("label_25"));
        label_25->setGeometry(QRect(224, 610, 39, 24));
        label_25->setFont(font1);
        freqLow = new QLineEdit(centralWidget);
        freqLow->setObjectName(QString::fromUtf8("freqLow"));
        freqLow->setGeometry(QRect(140, 610, 78, 28));
        freqLow->setFont(font2);
        layoutWidget = new QWidget(centralWidget);
        layoutWidget->setObjectName(QString::fromUtf8("layoutWidget"));
        layoutWidget->setGeometry(QRect(180, 730, 81, 30));
        obsPolXLayout = new QHBoxLayout(layoutWidget);
        obsPolXLayout->setSpacing(6);
        obsPolXLayout->setContentsMargins(11, 11, 11, 11);
        obsPolXLayout->setObjectName(QString::fromUtf8("obsPolXLayout"));
        obsPolXLayout->setContentsMargins(0, 0, 0, 0);
        obsPolX = new QLineEdit(layoutWidget);
        obsPolX->setObjectName(QString::fromUtf8("obsPolX"));
        obsPolX->setFont(font2);

        obsPolXLayout->addWidget(obsPolX);

        obsPolXDeg = new QLabel(layoutWidget);
        obsPolXDeg->setObjectName(QString::fromUtf8("obsPolXDeg"));
        obsPolXDeg->setFont(font3);

        obsPolXLayout->addWidget(obsPolXDeg);

        treeView = new QTreeView(centralWidget);
        treeView->setObjectName(QString::fromUtf8("treeView"));
        treeView->setGeometry(QRect(10, 80, 501, 241));
        importButton = new QPushButton(centralWidget);
        importButton->setObjectName(QString::fromUtf8("importButton"));
        importButton->setGeometry(QRect(280, 20, 141, 41));
        QFont font12;
        font12.setFamily(QString::fromUtf8("Microsoft JhengHei UI"));
        font12.setPointSize(12);
        font12.setBold(true);
        font12.setWeight(75);
        importButton->setFont(font12);
        importButton->setIcon(icon4);
        importCatButton = new QPushButton(centralWidget);
        importCatButton->setObjectName(QString::fromUtf8("importCatButton"));
        importCatButton->setGeometry(QRect(430, 20, 81, 41));
        importCatButton->setFont(font12);
        QIcon icon7;
        icon7.addFile(QString::fromUtf8(":/SBRApp/icon/importCat.png"), QSize(), QIcon::Normal, QIcon::Off);
        importCatButton->setIcon(icon7);
        SBRAppClass->setCentralWidget(centralWidget);
        label_27->raise();
        freqHigh->raise();
        label_23->raise();
        label_22->raise();
        label_24->raise();
        label_25->raise();
        freqLow->raise();
        label_14->raise();
        line_3->raise();
        label_15->raise();
        thetaIn->raise();
        label_16->raise();
        label_17->raise();
        phiIn->raise();
        label_18->raise();
        label_19->raise();
        inWavePol->raise();
        label_20->raise();
        obsPol->raise();
        label_21->raise();
        line_4->raise();
        rayPerLambda->raise();
        runMsg->raise();
        enterInTrCheck->raise();
        runButton->raise();
        resetButton->raise();
        cancelButton->raise();
        layoutWidget->raise();
        confirmButton->raise();
        meshWidget->raise();
        loadMsg->raise();
        freqLine->raise();
        label_28->raise();
        label_29->raise();
        label_26->raise();
        label_30->raise();
        label_31->raise();
        line_5->raise();
        tabWidget->raise();
        label_49->raise();
        rmPolX->raise();
        freqLineX->raise();
        line->raise();
        label->raise();
        label_2->raise();
        chartHead1->raise();
        chartHead2->raise();
        addPolX->raise();
        treeView->raise();
        importButton->raise();
        importCatButton->raise();
        menuBar = new QMenuBar(SBRAppClass);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 1920, 23));
        SBRAppClass->setMenuBar(menuBar);
        mainToolBar = new QToolBar(SBRAppClass);
        mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
        SBRAppClass->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(SBRAppClass);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        SBRAppClass->setStatusBar(statusBar);

        retranslateUi(SBRAppClass);

        tabWidget->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(SBRAppClass);
    } // setupUi

    void retranslateUi(QMainWindow *SBRAppClass)
    {
        SBRAppClass->setWindowTitle(QApplication::translate("SBRAppClass", "SBRApp", nullptr));
        label_14->setText(QApplication::translate("SBRAppClass", "Incident Wave Parameters", nullptr));
        label_15->setText(QApplication::translate("SBRAppClass", "Theta:", nullptr));
        label_16->setText(QApplication::translate("SBRAppClass", "\302\260", nullptr));
        label_17->setText(QApplication::translate("SBRAppClass", "Phi:", nullptr));
        label_18->setText(QApplication::translate("SBRAppClass", "\302\260", nullptr));
        label_19->setText(QApplication::translate("SBRAppClass", "Polar:", nullptr));
        label_20->setText(QApplication::translate("SBRAppClass", "Observed polarization", nullptr));
        label_21->setText(QApplication::translate("SBRAppClass", "Pulse Signal Parameters", nullptr));
        enterInTrCheck->setText(QApplication::translate("SBRAppClass", "tracking inside", nullptr));
        runButton->setText(QString());
#ifndef QT_NO_SHORTCUT
        runButton->setShortcut(QString());
#endif // QT_NO_SHORTCUT
        resetButton->setText(QString());
        cancelButton->setText(QApplication::translate("SBRAppClass", "Cancel", nullptr));
        confirmButton->setText(QApplication::translate("SBRAppClass", "Confirm Parameters", nullptr));
        label_28->setText(QString());
        label_29->setText(QString());
        label_26->setText(QApplication::translate("SBRAppClass", "\302\260", nullptr));
        label_30->setText(QString());
        label_31->setText(QApplication::translate("SBRAppClass", "\302\260", nullptr));
        label_32->setText(QApplication::translate("SBRAppClass", "    PEC (Perfect Electric Conductor) is assumed to be capable of carrying current and having infinite conductivity. \n"
"    PEC boundary conditions are used to simulate the behavior of an ideal conductor, assuming that electromagnetic waves do not induce electric fields on these surfaces, resulting in complete reflection of the electromagnetic waves. \n"
"    The current density on PEC surfaces is zero, and the electric field is perpendicular to the surface.", nullptr));
        pecImport->setText(QApplication::translate("SBRAppClass", "Import .stl", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab), QApplication::translate("SBRAppClass", "PEC", nullptr));
        label_33->setText(QApplication::translate("SBRAppClass", "    The coating material typically refers to a special material applied to a surface to alter the propagation characteristics of electromagnetic waves on it.\n"
"    Set the 'd (thickness)', 'mu', 'epsilon', then select an .STL mesh file as the coating material.", nullptr));
        coatImport->setText(QApplication::translate("SBRAppClass", "Import .stl", nullptr));
        label_35->setText(QApplication::translate("SBRAppClass", "<html><head/><body><p>permeability(\316\274) : </p></body></html>", nullptr));
        label_36->setText(QApplication::translate("SBRAppClass", "<html><head/><body><p>permittivity  (\316\265) : </p></body></html>", nullptr));
        label_37->setText(QApplication::translate("SBRAppClass", "+", nullptr));
        label_38->setText(QApplication::translate("SBRAppClass", " i", nullptr));
        label_39->setText(QApplication::translate("SBRAppClass", " i", nullptr));
        label_40->setText(QApplication::translate("SBRAppClass", "-", nullptr));
        label_47->setText(QApplication::translate("SBRAppClass", "<html><head/><body><p>d : </p></body></html>", nullptr));
        label_48->setText(QApplication::translate("SBRAppClass", "m", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_2), QApplication::translate("SBRAppClass", "Coating", nullptr));
        label_34->setText(QApplication::translate("SBRAppClass", "    The main characteristic of a dielectric is its ability to undergo polarization in an electric field without producing a significant electric current.\n"
"    Set the 'mu', 'epsilon', then select an .STL mesh file as the dielectric.", nullptr));
        dieleImport->setText(QApplication::translate("SBRAppClass", "Import .stl", nullptr));
        label_41->setText(QApplication::translate("SBRAppClass", " i", nullptr));
        label_42->setText(QApplication::translate("SBRAppClass", "+", nullptr));
        label_43->setText(QApplication::translate("SBRAppClass", "-", nullptr));
        label_44->setText(QApplication::translate("SBRAppClass", "<html><head/><body><p>permeability(\316\274) : </p></body></html>", nullptr));
        label_45->setText(QApplication::translate("SBRAppClass", " i", nullptr));
        label_46->setText(QApplication::translate("SBRAppClass", "<html><head/><body><p>permittivity  (\316\265) : </p></body></html>", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_3), QApplication::translate("SBRAppClass", "Dielectric", nullptr));
        label_49->setText(QApplication::translate("SBRAppClass", "Polar:", nullptr));
        addPolX->setText(QString());
        rmPolX->setText(QString());
        label->setText(QString());
        label_2->setText(QApplication::translate("SBRAppClass", "RCS", nullptr));
        chartHead1->setText(QApplication::translate("SBRAppClass", "TextLabel", nullptr));
        chartHead2->setText(QApplication::translate("SBRAppClass", "TextLabel", nullptr));
        label_27->setText(QApplication::translate("SBRAppClass", "rays/lambda:", nullptr));
        label_23->setText(QApplication::translate("SBRAppClass", "GHz", nullptr));
        label_22->setText(QApplication::translate("SBRAppClass", "Maximum :", nullptr));
        label_24->setText(QApplication::translate("SBRAppClass", "Minimum  :", nullptr));
        label_25->setText(QApplication::translate("SBRAppClass", "GHz", nullptr));
        obsPolXDeg->setText(QApplication::translate("SBRAppClass", "\302\260", nullptr));
        importButton->setText(QApplication::translate("SBRAppClass", "IMPORT", nullptr));
        importCatButton->setText(QApplication::translate("SBRAppClass", "ADD", nullptr));
    } // retranslateUi

};

namespace Ui {
    class SBRAppClass: public Ui_SBRAppClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SBRAPP_H
