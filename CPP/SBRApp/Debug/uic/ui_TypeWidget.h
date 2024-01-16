/********************************************************************************
** Form generated from reading UI file 'TypeWidget.ui'
**
** Created by: Qt User Interface Compiler version 5.12.12
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_TYPEWIDGET_H
#define UI_TYPEWIDGET_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_TypeWidgetClass
{
public:
    QWidget *centralWidget;
    QLabel *label;
    QTabWidget *tabWidget;
    QWidget *tab;
    QLabel *label_32;
    QWidget *tab_2;
    QLabel *label_33;
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
    QPushButton *coatOkBtn;
    QPushButton *coatSaveBtn;
    QWidget *tab_3;
    QLabel *label_34;
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
    QPushButton *dieOkBtn;
    QPushButton *dieSaveBtn;
    QLineEdit *typeName;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *TypeWidgetClass)
    {
        if (TypeWidgetClass->objectName().isEmpty())
            TypeWidgetClass->setObjectName(QString::fromUtf8("TypeWidgetClass"));
        TypeWidgetClass->resize(464, 435);
        centralWidget = new QWidget(TypeWidgetClass);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        label = new QLabel(centralWidget);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(20, 10, 71, 31));
        QFont font;
        font.setFamily(QString::fromUtf8("Microsoft JhengHei UI"));
        font.setPointSize(14);
        font.setBold(true);
        font.setWeight(75);
        label->setFont(font);
        tabWidget = new QTabWidget(centralWidget);
        tabWidget->setObjectName(QString::fromUtf8("tabWidget"));
        tabWidget->setGeometry(QRect(10, 60, 431, 321));
        tabWidget->setStyleSheet(QString::fromUtf8("font: 14pt \"Microsoft JhengHei UI\";"));
        tab = new QWidget();
        tab->setObjectName(QString::fromUtf8("tab"));
        label_32 = new QLabel(tab);
        label_32->setObjectName(QString::fromUtf8("label_32"));
        label_32->setGeometry(QRect(10, 0, 401, 221));
        QFont font1;
        font1.setFamily(QString::fromUtf8("Times New Roman"));
        font1.setPointSize(12);
        font1.setBold(false);
        font1.setItalic(false);
        font1.setWeight(50);
        label_32->setFont(font1);
        label_32->setStyleSheet(QString::fromUtf8("font: 12pt \"Times New Roman\";"));
        label_32->setWordWrap(true);
        tabWidget->addTab(tab, QString());
        tab_2 = new QWidget();
        tab_2->setObjectName(QString::fromUtf8("tab_2"));
        label_33 = new QLabel(tab_2);
        label_33->setObjectName(QString::fromUtf8("label_33"));
        label_33->setGeometry(QRect(10, 0, 391, 161));
        label_33->setFont(font1);
        label_33->setStyleSheet(QString::fromUtf8("font: 12pt \"Times New Roman\";"));
        label_33->setWordWrap(true);
        label_35 = new QLabel(tab_2);
        label_35->setObjectName(QString::fromUtf8("label_35"));
        label_35->setGeometry(QRect(20, 160, 121, 31));
        QFont font2;
        font2.setFamily(QString::fromUtf8("Times New Roman"));
        font2.setPointSize(13);
        font2.setBold(false);
        font2.setItalic(false);
        font2.setWeight(50);
        label_35->setFont(font2);
        label_35->setStyleSheet(QString::fromUtf8("font: 13pt \"Times New Roman\";"));
        label_36 = new QLabel(tab_2);
        label_36->setObjectName(QString::fromUtf8("label_36"));
        label_36->setGeometry(QRect(20, 190, 121, 31));
        label_36->setFont(font2);
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
        label_47->setFont(font2);
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
        coatOkBtn = new QPushButton(tab_2);
        coatOkBtn->setObjectName(QString::fromUtf8("coatOkBtn"));
        coatOkBtn->setGeometry(QRect(200, 240, 101, 31));
        coatSaveBtn = new QPushButton(tab_2);
        coatSaveBtn->setObjectName(QString::fromUtf8("coatSaveBtn"));
        coatSaveBtn->setGeometry(QRect(310, 240, 101, 31));
        tabWidget->addTab(tab_2, QString());
        tab_3 = new QWidget();
        tab_3->setObjectName(QString::fromUtf8("tab_3"));
        label_34 = new QLabel(tab_3);
        label_34->setObjectName(QString::fromUtf8("label_34"));
        label_34->setGeometry(QRect(10, 0, 391, 151));
        label_34->setFont(font1);
        label_34->setStyleSheet(QString::fromUtf8("font: 12pt \"Times New Roman\";"));
        label_34->setWordWrap(true);
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
        label_44->setFont(font2);
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
        label_46->setFont(font2);
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
        dieOkBtn = new QPushButton(tab_3);
        dieOkBtn->setObjectName(QString::fromUtf8("dieOkBtn"));
        dieOkBtn->setGeometry(QRect(200, 240, 101, 31));
        dieSaveBtn = new QPushButton(tab_3);
        dieSaveBtn->setObjectName(QString::fromUtf8("dieSaveBtn"));
        dieSaveBtn->setGeometry(QRect(310, 240, 101, 31));
        tabWidget->addTab(tab_3, QString());
        typeName = new QLineEdit(centralWidget);
        typeName->setObjectName(QString::fromUtf8("typeName"));
        typeName->setGeometry(QRect(90, 10, 121, 31));
        TypeWidgetClass->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(TypeWidgetClass);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 464, 23));
        TypeWidgetClass->setMenuBar(menuBar);
        mainToolBar = new QToolBar(TypeWidgetClass);
        mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
        TypeWidgetClass->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(TypeWidgetClass);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        TypeWidgetClass->setStatusBar(statusBar);

        retranslateUi(TypeWidgetClass);

        tabWidget->setCurrentIndex(2);


        QMetaObject::connectSlotsByName(TypeWidgetClass);
    } // setupUi

    void retranslateUi(QMainWindow *TypeWidgetClass)
    {
        TypeWidgetClass->setWindowTitle(QApplication::translate("TypeWidgetClass", "TypeWidget", nullptr));
        label->setText(QApplication::translate("TypeWidgetClass", "Name: ", nullptr));
        label_32->setText(QApplication::translate("TypeWidgetClass", "    PEC (Perfect Electric Conductor) is assumed to be capable of carrying current and having infinite conductivity. \n"
"    PEC boundary conditions are used to simulate the behavior of an ideal conductor, assuming that electromagnetic waves do not induce electric fields on these surfaces, resulting in complete reflection of the electromagnetic waves. \n"
"    The current density on PEC surfaces is zero, and the electric field is perpendicular to the surface.", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab), QApplication::translate("TypeWidgetClass", "PEC", nullptr));
        label_33->setText(QApplication::translate("TypeWidgetClass", "    The coating material typically refers to a special material applied to a surface to alter the propagation characteristics of electromagnetic waves on it.\n"
"    Set the 'd (thickness)', 'mu', 'epsilon', then select an .STL mesh file as the coating material.", nullptr));
        label_35->setText(QApplication::translate("TypeWidgetClass", "<html><head/><body><p>permeability(\316\274) : </p></body></html>", nullptr));
        label_36->setText(QApplication::translate("TypeWidgetClass", "<html><head/><body><p>permittivity  (\316\265) : </p></body></html>", nullptr));
        label_37->setText(QApplication::translate("TypeWidgetClass", "+", nullptr));
        label_38->setText(QApplication::translate("TypeWidgetClass", " i", nullptr));
        label_39->setText(QApplication::translate("TypeWidgetClass", " i", nullptr));
        label_40->setText(QApplication::translate("TypeWidgetClass", "+", nullptr));
        label_47->setText(QApplication::translate("TypeWidgetClass", "<html><head/><body><p>d : </p></body></html>", nullptr));
        label_48->setText(QApplication::translate("TypeWidgetClass", "m", nullptr));
        coatOkBtn->setText(QApplication::translate("TypeWidgetClass", "Add", nullptr));
        coatSaveBtn->setText(QApplication::translate("TypeWidgetClass", "Save", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_2), QApplication::translate("TypeWidgetClass", "Coating", nullptr));
        label_34->setText(QApplication::translate("TypeWidgetClass", "    The main characteristic of a dielectric is its ability to undergo polarization in an electric field without producing a significant electric current.\n"
"    Set the 'mu', 'epsilon', then select an .STL mesh file as the dielectric.", nullptr));
        label_41->setText(QApplication::translate("TypeWidgetClass", " i", nullptr));
        label_42->setText(QApplication::translate("TypeWidgetClass", "+", nullptr));
        label_43->setText(QApplication::translate("TypeWidgetClass", "+", nullptr));
        label_44->setText(QApplication::translate("TypeWidgetClass", "<html><head/><body><p>permeability(\316\274) : </p></body></html>", nullptr));
        label_45->setText(QApplication::translate("TypeWidgetClass", " i", nullptr));
        label_46->setText(QApplication::translate("TypeWidgetClass", "<html><head/><body><p>permittivity  (\316\265) : </p></body></html>", nullptr));
        dieOkBtn->setText(QApplication::translate("TypeWidgetClass", "Add", nullptr));
        dieSaveBtn->setText(QApplication::translate("TypeWidgetClass", "Save", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_3), QApplication::translate("TypeWidgetClass", "Dielectric", nullptr));
    } // retranslateUi

};

namespace Ui {
    class TypeWidgetClass: public Ui_TypeWidgetClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_TYPEWIDGET_H
