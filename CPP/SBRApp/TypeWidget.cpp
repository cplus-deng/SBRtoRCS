#include "TypeWidget.h"
#include <QMessageBox>
TypeWidget::TypeWidget(QWidget *parent)
	: QMainWindow(parent)
	, ui(new Ui::TypeWidgetClass())
{
	ui->setupUi(this);
    ui->coatOkBtn->setEnabled(true);
    ui->dieOkBtn->setEnabled(true);
    ui->coatSaveBtn->setEnabled(false);
    ui->dieSaveBtn->setEnabled(false);
    connect(ui->coatOkBtn, &QPushButton::clicked, this, &TypeWidget::onCoatOkBtnClicked);
    connect(ui->dieOkBtn, &QPushButton::clicked, this, &TypeWidget::onDieOkBtnClicked);

    connect(ui->coatSaveBtn, &QPushButton::clicked, this, &TypeWidget::onCoatSaveBtnClicked);
    connect(ui->dieSaveBtn, &QPushButton::clicked, this, &TypeWidget::onDieSaveBtnClicked);
}

TypeWidget::~TypeWidget()
{
	delete ui;
}

void TypeWidget::receiveEditTypeFromMainWin(QStringList data)
{
    ui->coatOkBtn->setEnabled(false);
    ui->dieOkBtn->setEnabled(false);
    ui->coatSaveBtn->setEnabled(true);
    ui->dieSaveBtn->setEnabled(true);

    if (data.size() == 5) //½éÖÊ²ÄÁÏ
    {
        ui->typeName->setText(data.at(0));
        ui->dieMuR->setText(data.at(1));
        ui->dieMuI->setText(data.at(2));
        ui->dieEpsR->setText(data.at(3));
        ui->dieEpsI->setText(data.at(4));

        ui->coatMuR->clear();
        ui->coatMuI->clear();
        ui->coatEpsR->clear();
        ui->coatEpsI->clear();
        ui->coatThickIn->clear();

        //ui->tabWidget->setCurrentIndex(1);

    }
    else if (data.size() == 6) //Í¿²ã²ÄÁÏ
    {
        ui->typeName->setText(data.at(0));
        ui->coatMuR->setText(data.at(1));
        ui->coatMuI->setText(data.at(2));
        ui->coatEpsR->setText(data.at(3));
        ui->coatEpsI->setText(data.at(4));
        ui->coatThickIn->setText(data.at(5));

        ui->dieMuR->clear();
        ui->dieMuI->clear();
        ui->dieEpsR->clear();
        ui->dieEpsI->clear();

        //ui->tabWidget->setCurrentIndex(2);
    }
}
void TypeWidget::onCoatOkBtnClicked()
{
    bool ok1, ok2, ok3, ok4, ok5, ok6;
    Float muRealPart = ui->coatMuR->text().toFloat(&ok1);
    Float muImagePart = ui->coatMuI->text().toFloat(&ok2);
    Float epsRealPart = ui->coatEpsR->text().toFloat(&ok3);
    Float epsImagePart = ui->coatEpsI->text().toFloat(&ok4);
    Float coatD = ui->coatThickIn->text().toFloat(&ok5);

    ok6 = ui->typeName->text().isEmpty();
    if (ok1 && ok2 && ok3 && ok4 && ok5 && !ok6)
    {
        QStringList data;
        data.append(ui->typeName->text());
        data.append(ui->coatMuR->text()); data.append(ui->coatMuI->text());
        data.append(ui->coatEpsR->text()); data.append(ui->coatEpsI->text());
        data.append(ui->coatThickIn->text());

        emit sendNewTypeToMainWin(data);
    }

    else
    {
        QMessageBox::information(this, "Information", "Invalid Input.");
    }
}

void TypeWidget::onCoatSaveBtnClicked()
{
    bool ok1, ok2, ok3, ok4, ok5, ok6;
    Float muRealPart = ui->coatMuR->text().toFloat(&ok1);
    Float muImagePart = ui->coatMuI->text().toFloat(&ok2);
    Float epsRealPart = ui->coatEpsR->text().toFloat(&ok3);
    Float epsImagePart = ui->coatEpsI->text().toFloat(&ok4);
    Float coatD = ui->coatThickIn->text().toFloat(&ok5);

    ok6 = ui->typeName->text().isEmpty();
    if (ok1 && ok2 && ok3 && ok4 && ok5 && !ok6)
    {
        QStringList data;
        data.append(ui->typeName->text());
        data.append(ui->coatMuR->text()); data.append(ui->coatMuI->text());
        data.append(ui->coatEpsR->text()); data.append(ui->coatEpsI->text());
        data.append(ui->coatThickIn->text());

        emit sendEditTypeToMainWin(data);
        this->close();
    }

    else
    {
        QMessageBox::information(this, "Information", "Invalid Input.");
    }
}
void TypeWidget::onDieOkBtnClicked()
{
    bool ok1, ok2, ok3, ok4, ok5;
    Float muRealPart = ui->dieMuR->text().toFloat(&ok1);
    Float muImagePart = ui->dieMuI->text().toFloat(&ok2);
    Float epsRealPart = ui->dieEpsR->text().toFloat(&ok3);
    Float epsImagePart = ui->dieEpsI->text().toFloat(&ok4);
    ok5 = ui->typeName->text().isEmpty();
    if (ok1 && ok2 && ok3 && ok4 && !ok5)
    {
        QStringList data;
        data.append(ui->typeName->text());
        data.append(ui->dieMuR->text()); data.append(ui->dieMuI->text());
        data.append(ui->dieEpsR->text()); data.append(ui->dieEpsI->text());
        emit sendNewTypeToMainWin(data);
    }

    else 
    {
        QMessageBox::information(this, "Information", "Invalid Input.");
    }
}

void TypeWidget::onDieSaveBtnClicked()
{
    bool ok1, ok2, ok3, ok4, ok5;
    Float muRealPart = ui->dieMuR->text().toFloat(&ok1);
    Float muImagePart = ui->dieMuI->text().toFloat(&ok2);
    Float epsRealPart = ui->dieEpsR->text().toFloat(&ok3);
    Float epsImagePart = ui->dieEpsI->text().toFloat(&ok4);
    ok5 = ui->typeName->text().isEmpty();
    if (ok1 && ok2 && ok3 && ok4 && !ok5)
    {
        QStringList data;
        data.append(ui->typeName->text());
        data.append(ui->dieMuR->text()); data.append(ui->dieMuI->text());
        data.append(ui->dieEpsR->text()); data.append(ui->dieEpsI->text());
        emit sendEditTypeToMainWin(data);
        this->close();
    }

    else
    {
        QMessageBox::information(this, "Information", "Invalid Input.");
    }
}

