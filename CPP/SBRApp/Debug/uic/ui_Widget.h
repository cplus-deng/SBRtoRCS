/********************************************************************************
** Form generated from reading UI file 'Widget.ui'
**
** Created by: Qt User Interface Compiler version 5.12.12
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_WIDGET_H
#define UI_WIDGET_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_Form
{
public:
    QTabWidget *tabWidget;
    QWidget *tab;
    QLabel *label_32;
    QPushButton *pushButton;
    QPushButton *pushButton_2;
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
    QPushButton *pushButton_3;
    QPushButton *pushButton_4;
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
    QPushButton *pushButton_5;
    QPushButton *pushButton_6;
    QLabel *label;
    QLabel *label_2;

    void setupUi(QWidget *Form)
    {
        if (Form->objectName().isEmpty())
            Form->setObjectName(QString::fromUtf8("Form"));
        Form->resize(435, 816);
        tabWidget = new QTabWidget(Form);
        tabWidget->setObjectName(QString::fromUtf8("tabWidget"));
        tabWidget->setGeometry(QRect(0, 90, 431, 321));
        tabWidget->setStyleSheet(QString::fromUtf8("font: 14pt \"Microsoft JhengHei UI\";"));
        tab = new QWidget();
        tab->setObjectName(QString::fromUtf8("tab"));
        label_32 = new QLabel(tab);
        label_32->setObjectName(QString::fromUtf8("label_32"));
        label_32->setGeometry(QRect(10, 0, 401, 221));
        QFont font;
        font.setFamily(QString::fromUtf8("Times New Roman"));
        font.setPointSize(12);
        font.setBold(false);
        font.setItalic(false);
        font.setWeight(50);
        label_32->setFont(font);
        label_32->setStyleSheet(QString::fromUtf8("font: 12pt \"Times New Roman\";"));
        label_32->setWordWrap(true);
        pushButton = new QPushButton(tab);
        pushButton->setObjectName(QString::fromUtf8("pushButton"));
        pushButton->setGeometry(QRect(200, 240, 101, 31));
        pushButton_2 = new QPushButton(tab);
        pushButton_2->setObjectName(QString::fromUtf8("pushButton_2"));
        pushButton_2->setGeometry(QRect(310, 240, 101, 31));
        tabWidget->addTab(tab, QString());
        tab_2 = new QWidget();
        tab_2->setObjectName(QString::fromUtf8("tab_2"));
        label_33 = new QLabel(tab_2);
        label_33->setObjectName(QString::fromUtf8("label_33"));
        label_33->setGeometry(QRect(10, 0, 391, 161));
        label_33->setFont(font);
        label_33->setStyleSheet(QString::fromUtf8("font: 12pt \"Times New Roman\";"));
        label_33->setWordWrap(true);
        label_35 = new QLabel(tab_2);
        label_35->setObjectName(QString::fromUtf8("label_35"));
        label_35->setGeometry(QRect(20, 160, 121, 31));
        QFont font1;
        font1.setFamily(QString::fromUtf8("Times New Roman"));
        font1.setPointSize(13);
        font1.setBold(false);
        font1.setItalic(false);
        font1.setWeight(50);
        label_35->setFont(font1);
        label_35->setStyleSheet(QString::fromUtf8("font: 13pt \"Times New Roman\";"));
        label_36 = new QLabel(tab_2);
        label_36->setObjectName(QString::fromUtf8("label_36"));
        label_36->setGeometry(QRect(20, 190, 121, 31));
        label_36->setFont(font1);
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
        label_47->setFont(font1);
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
        pushButton_3 = new QPushButton(tab_2);
        pushButton_3->setObjectName(QString::fromUtf8("pushButton_3"));
        pushButton_3->setGeometry(QRect(200, 240, 101, 31));
        pushButton_4 = new QPushButton(tab_2);
        pushButton_4->setObjectName(QString::fromUtf8("pushButton_4"));
        pushButton_4->setGeometry(QRect(310, 240, 101, 31));
        tabWidget->addTab(tab_2, QString());
        tab_3 = new QWidget();
        tab_3->setObjectName(QString::fromUtf8("tab_3"));
        label_34 = new QLabel(tab_3);
        label_34->setObjectName(QString::fromUtf8("label_34"));
        label_34->setGeometry(QRect(10, 0, 391, 151));
        label_34->setFont(font);
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
        label_44->setFont(font1);
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
        label_46->setFont(font1);
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
        pushButton_5 = new QPushButton(tab_3);
        pushButton_5->setObjectName(QString::fromUtf8("pushButton_5"));
        pushButton_5->setGeometry(QRect(200, 240, 101, 31));
        pushButton_6 = new QPushButton(tab_3);
        pushButton_6->setObjectName(QString::fromUtf8("pushButton_6"));
        pushButton_6->setGeometry(QRect(310, 240, 101, 31));
        tabWidget->addTab(tab_3, QString());
        label = new QLabel(Form);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(10, 10, 411, 31));
        QFont font2;
        font2.setFamily(QString::fromUtf8("Microsoft JhengHei UI"));
        font2.setPointSize(14);
        font2.setBold(true);
        font2.setWeight(75);
        label->setFont(font2);
        label_2 = new QLabel(Form);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setGeometry(QRect(20, 40, 411, 31));
        label_2->setFont(font2);

        retranslateUi(Form);

        tabWidget->setCurrentIndex(2);


        QMetaObject::connectSlotsByName(Form);
    } // setupUi

    void retranslateUi(QWidget *Form)
    {
        Form->setWindowTitle(QApplication::translate("Form", "Form", nullptr));
        label_32->setText(QApplication::translate("Form", "    PEC (Perfect Electric Conductor) is assumed to be capable of carrying current and having infinite conductivity. \n"
"    PEC boundary conditions are used to simulate the behavior of an ideal conductor, assuming that electromagnetic waves do not induce electric fields on these surfaces, resulting in complete reflection of the electromagnetic waves. \n"
"    The current density on PEC surfaces is zero, and the electric field is perpendicular to the surface.", nullptr));
        pushButton->setText(QApplication::translate("Form", "OK", nullptr));
        pushButton_2->setText(QApplication::translate("Form", "Cancel", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab), QApplication::translate("Form", "PEC", nullptr));
        label_33->setText(QApplication::translate("Form", "    The coating material typically refers to a special material applied to a surface to alter the propagation characteristics of electromagnetic waves on it.\n"
"    Set the 'd (thickness)', 'mu', 'epsilon', then select an .STL mesh file as the coating material.", nullptr));
        label_35->setText(QApplication::translate("Form", "<html><head/><body><p>permeability(\316\274) : </p></body></html>", nullptr));
        label_36->setText(QApplication::translate("Form", "<html><head/><body><p>permittivity  (\316\265) : </p></body></html>", nullptr));
        label_37->setText(QApplication::translate("Form", "+", nullptr));
        label_38->setText(QApplication::translate("Form", " i", nullptr));
        label_39->setText(QApplication::translate("Form", " i", nullptr));
        label_40->setText(QApplication::translate("Form", "-", nullptr));
        label_47->setText(QApplication::translate("Form", "<html><head/><body><p>d : </p></body></html>", nullptr));
        label_48->setText(QApplication::translate("Form", "m", nullptr));
        pushButton_3->setText(QApplication::translate("Form", "OK", nullptr));
        pushButton_4->setText(QApplication::translate("Form", "Cancel", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_2), QApplication::translate("Form", "Coating", nullptr));
        label_34->setText(QApplication::translate("Form", "    The main characteristic of a dielectric is its ability to undergo polarization in an electric field without producing a significant electric current.\n"
"    Set the 'mu', 'epsilon', then select an .STL mesh file as the dielectric.", nullptr));
        label_41->setText(QApplication::translate("Form", " i", nullptr));
        label_42->setText(QApplication::translate("Form", "+", nullptr));
        label_43->setText(QApplication::translate("Form", "-", nullptr));
        label_44->setText(QApplication::translate("Form", "<html><head/><body><p>permeability(\316\274) : </p></body></html>", nullptr));
        label_45->setText(QApplication::translate("Form", " i", nullptr));
        label_46->setText(QApplication::translate("Form", "<html><head/><body><p>permittivity  (\316\265) : </p></body></html>", nullptr));
        pushButton_5->setText(QApplication::translate("Form", "OK", nullptr));
        pushButton_6->setText(QApplication::translate("Form", "Cancel", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_3), QApplication::translate("Form", "Dielectric", nullptr));
        label->setText(QApplication::translate("Form", "Name: solid1", nullptr));
        label_2->setText(QApplication::translate("Form", "Type : PEC", nullptr));
    } // retranslateUi

};

namespace Ui {
    class Form: public Ui_Form {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_WIDGET_H
