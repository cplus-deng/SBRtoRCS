#ifndef _SILENCE_AMP_DEPRECATION_WARNINGS
#define _SILENCE_AMP_DEPRECATION_WARNINGS
#endif // !_SILENCE_AMP_DEPRECATION_WARNINGS

#define ROLE_MARK ( Qt::UserRole + 1 )
#define ROLE_MARK_FOLDER ( Qt::UserRole + 2 )
#define ROLE_MARK_ITEM ( Qt::UserRole + 3 )

#define MARK_FOLDER 1
#define MARK_ITEM 2

#define MARK_FOLDER_SOLID 1
#define MARK_FOLDER_TYPE 2

#define MARK_ITEM_SOLID 1
#define MARK_ITEM_TYPE 2

#include <iostream>
#include <QThread>
#include <QtCharts>
#include "StlTrigMeshFile.h"
#include "TypeDef.hpp"
#include "Observation.hpp"
//#include "Triangle.hpp"
//#include "TriangleMesh.hpp"
//#include "MortonManager.hpp"
//#include "BvhGenerator.hpp"
//#include "BvhNodeTypes.hpp"
//#include "ReducedBvhArray.hpp"
//#include "RayPool.hpp"
//#include "DepthMapGenerator.hpp"
//#include "SbrSolver.hpp"
//#include "Excitation.h"
#include "STLParser.h"

#include "SBRApp.h"

#include <QFileDialog>
#include <QFileInfo>

SBRApp::SBRApp(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::SBRAppClass())
{
    ui->setupUi(this);
    initTreeView();
    const Float pi_ = 3.1415926;
    ui->runMsg->insertPlainText("###Start SBR###");
    auto tStart = Clock::now();
    readyParams = false;
    readyMesh = false;
    //ui->runButton->setEnabled(false);
    U32 metalType = 0;
    U32 coatintType = 100;
    U32 dielectType = 200;
    /*std::vector< Param< Float > > coatingParams;
    std::vector< Param< Float > > dielectParams;

    std::vector< std::string > metalMeshVecs;
    std::vector< std::string > coatingMeshVecs;
    std::vector< std::string > dielectMeshVecs;*/
    connect(ui->importButton, &QPushButton::clicked, this, &SBRApp::onImport);
    connect(ui->pecImport, SIGNAL(clicked()), this, SLOT(onAddMetal()));
    //-----------------Coating Button
    ui->coatImport->setEnabled(false);
    connect(ui->coatThickIn, SIGNAL(textChanged(const QString&)), this, SLOT(onHandleCoatingParams()));
    connect(ui->coatMuR, SIGNAL(textChanged(const QString&)), this, SLOT(onHandleCoatingParams()));
    connect(ui->coatMuI, SIGNAL(textChanged(const QString&)), this, SLOT(onHandleCoatingParams()));
    connect(ui->coatEpsR, SIGNAL(textChanged(const QString&)), this, SLOT(onHandleCoatingParams()));
    connect(ui->coatEpsI, SIGNAL(textChanged(const QString&)), this, SLOT(onHandleCoatingParams()));

    //-----------------Dielect Button
    ui->dieleImport->setEnabled(false);
    connect(ui->dieMuR, SIGNAL(textChanged(const QString&)), this, SLOT(onHandleDielectParams()));
    connect(ui->dieMuI, SIGNAL(textChanged(const QString&)), this, SLOT(onHandleDielectParams()));
    connect(ui->dieEpsR, SIGNAL(textChanged(const QString&)), this, SLOT(onHandleDielectParams()));
    connect(ui->dieEpsI, SIGNAL(textChanged(const QString&)), this, SLOT(onHandleDielectParams()));

    connect(ui->coatImport, SIGNAL(clicked()), this, SLOT(onAddCoating()));
    connect(ui->dieleImport, SIGNAL(clicked()), this, SLOT(onAddDie()));
    //ui->enterInTrCheck->setEnabled(false);
    ui->enterInTrCheck->setToolTip("whether rays penetrate into the interior of the dielectric.");
    //connect(ui->confirmImButton, SIGNAL(clicked()), this, SLOT());
    ui->obsPolX->setVisible(false); ui->obsPolXDeg->setVisible(false); ui->rmPolX->setVisible(false);

    connect(ui->confirmButton, SIGNAL(clicked()), this, SLOT(onConfirmParams()));
    connect(ui->runButton, &QPushButton::clicked, this, &SBRApp::onRunBtnClicked);

    //3D Tab Widgets
    /*****************************************/
    QGridLayout* layout3DView = new QGridLayout(ui->meshWidget);
    glView = new AppGLWidget(ui->meshWidget);

    layout3DView->addWidget(glView);
    ui->meshWidget->setLayout(layout3DView);
    /*****************************************/

    /*
    * Add another obs pol
    */
    polObsCnt = 1;
    connect(ui->addPolX, &QPushButton::clicked, this, &SBRApp::addPol);
    connect(ui->rmPolX, &QPushButton::clicked, this, &SBRApp::rmPol);

    ui->freqLine->setVisible(false); ui->freqLineX->setVisible(false);


    connect(this, &SBRApp::sendEditTypeToDialog, &dialog, &TypeWidget::receiveEditTypeFromMainWin);
    connect(&dialog, &TypeWidget::sendNewTypeToMainWin, this, &SBRApp::receiveNewTypeFromDialog);
    connect(&dialog, &TypeWidget::sendEditTypeToMainWin, this, &SBRApp::receiveEditTypeFromDialog);
}

SBRApp::~SBRApp()
{
    delete ui;
}

void SBRApp::initTreeView()
{
    ui->treeView->setEditTriggers(QTreeView::NoEditTriggers);
    model = new QStandardItemModel(ui->treeView);

    solidFolder = new QStandardItem(QIcon(":/SBRApp/icon/folder.png"), QStringLiteral("Solids"));
    solidFolder->setData(MARK_FOLDER, ROLE_MARK);
    solidFolder->setData(MARK_FOLDER_SOLID, ROLE_MARK_FOLDER);

    typeFolder = new QStandardItem(QIcon(":/SBRApp/icon/type.png"), QStringLiteral("Types"));
    typeFolder->setData(MARK_FOLDER, ROLE_MARK);
    typeFolder->setData(MARK_FOLDER_TYPE, ROLE_MARK_FOLDER);

    model->appendRow(solidFolder);
    model->appendRow(typeFolder);

    MType pecType;
    pecType.name_ = std::string("PEC");
    pecType.type_ = SolidType::PEC;
    types.push_back(pecType);

    QStandardItem* m_PEC = new QStandardItem(QIcon(":/SBRApp/icon/PEC.png"), QString("PEC"));
    m_PEC->setData(MARK_ITEM, ROLE_MARK);
    m_PEC->setData(MARK_ITEM_TYPE, ROLE_MARK_ITEM);
    typeFolder->appendRow(m_PEC);
    ui->treeView->header()->hide();
    ui->treeView->setModel(model);


    ui->treeView->setContextMenuPolicy(Qt::CustomContextMenu);
    connect(ui->treeView, &QTreeView::customContextMenuRequested, this, &SBRApp::onTreeMenu);

    /*QAction* ac = nullptr;*/
    solidFolderMenu = new QMenu(this); //物体文件夹
    QAction* ac1 = new QAction(QIcon(":/SBRApp/icon/delete.png"),"Clear", this);
    solidFolderMenu->addAction(ac1);
    connect(ac1, &QAction::triggered, this, &SBRApp::clearAllSolids);

    solidItemMenu = new QMenu(this);  //单个物体
    QAction* ac2 = new QAction(QIcon(":/SBRApp/icon/type.png"), "Type", this);
    //connect(ac2, &QAction::triggered, this, &SBRApp::updateSolidTypeList);
    solidItemMenu->addAction(ac2);
    selectType = ac2;
    QAction* ac3 = new QAction(QIcon(":/SBRApp/icon/delete.png"), "Remove", this);
    connect(ac3, &QAction::triggered, this, &SBRApp::onRemoveSolidItem);
    solidItemMenu->addAction(ac3);

    typeFolderMenu = new QMenu(this);  //材料文件夹
    QAction* ac4 = new QAction(QIcon(":/SBRApp/icon/add.png"), "Add", this);
    connect(ac4, &QAction::triggered, this, &SBRApp::onAddType);
    typeFolderMenu->addAction(ac4);
    QAction* ac5 = new QAction(QIcon(":/SBRApp/icon/delete.png"), "Clear", this);
    typeFolderMenu->addAction(ac5); 
    connect(ac4, &QAction::triggered, this, &SBRApp::onAddType);
    connect(ac5, &QAction::triggered, this, &SBRApp::clearAllTypes);
    
    typeItemMenu = new QMenu(this);  //单个材料
    QAction* ac6 = new QAction(QIcon(":/SBRApp/icon/edit.png"), "Edit", this);
    connect(ac6, &QAction::triggered, this, &SBRApp::onEditType);
    typeItemMenu->addAction(ac6);
    QAction* ac7 = new QAction(QIcon(":/SBRApp/icon/delete.png"), "Remove", this);
    connect(ac7, &QAction::triggered, this, &SBRApp::onRemoveTypeItem);
    typeItemMenu->addAction(ac7);

    solidItemSubMenu = new QMenu(solidItemMenu);
    mapper = new QSignalMapper(this);
    connect(mapper, SIGNAL(mapped(int)), this, SLOT(setSolidType(int)));
}
void SBRApp::onTreeMenu(const QPoint& pos)
{
    updateSolidTypeList(pos);
    QModelIndex curIndex = ui->treeView->indexAt(pos);
    QVariant var = curIndex.data(ROLE_MARK);
    if (var.isValid())
    {
        if (MARK_FOLDER == var.toInt())
        {
            QVariant varIn = curIndex.data(ROLE_MARK_FOLDER);
            if (var.isValid())
            {
                if (MARK_FOLDER_SOLID == varIn.toInt())
                {
                    solidFolderMenu->exec(QCursor::pos());
                }
                else if (MARK_FOLDER_TYPE == varIn.toInt())
                {
                    typeFolderMenu->exec(QCursor::pos());
                }
            }
        }
        else if (MARK_ITEM == var.toInt())
        {
            QVariant varIn = curIndex.data(ROLE_MARK_ITEM);
            if (var.isValid())
            {
                if (MARK_ITEM_SOLID == varIn.toInt())
                {
                    solidItemMenu->exec(QCursor::pos());
                    qDebug() << "row:" << curIndex.row();
                }
                else if (MARK_ITEM_TYPE == varIn.toInt())
                {
                    typeItemMenu->exec(QCursor::pos());
                }
            }
        }
    }
}
void SBRApp::reLoadGL()
{
    m_.Reset(); 
    if (solids.empty()) {
        glView->SetModel(m_);
    }
    glView->SetModel(m_);
    for (int idx = 0; idx < solids.size(); idx++)
    {
        loadGL(QString::fromStdString(solids[idx].path_));
    }
    ui->loadMsg->setText(QString::fromStdString(std::to_string(solids.size())) + " solid(s) loaded.");
}
void SBRApp::loadGL(const QString& meshFileName)
{
    STLParser parser;
    if (!meshFileName.isEmpty()) {
        QFile meshFile(meshFileName);
        if (!meshFile.open(QIODevice::ReadOnly)) {
            ui->loadMsg->setText("Cannot open " + meshFileName);
            return;
        }
        //Model m = parser.parse(meshFile);
        parser.parseAppend(m_, meshFile);
        if (m_.isInitialized()) {
            glView->SetModel(m_);
            glView->show();
            ui->loadMsg->setText(QFileInfo(meshFileName).fileName() + " loaded.");
        }
        else
        {
            ui->loadMsg->setText("Invalid ASCII .stl file.");
        }
    }
}
void SBRApp::onAddMetal()
{
    QString path = "E:\\Files";
    QString filter = "STL Files (*.stl)";
    QString meshFileName = QFileDialog::getOpenFileName(this, "Load ASCII .stl model(For Metal)", path, filter);
    if (!meshFileName.isEmpty()) {
        metalMeshVecs.push_back(meshFileName.toStdString());
        ui->runMsg->insertPlainText("\n#Select " + meshFileName + " as Metal.");
        ui->loadMsg->setText("Loading " + meshFileName);
        loadGL(meshFileName);
        readyMesh = true;
        
        /*
        if (readyMesh && readyParams)
        {
            ui->runButton->setEnabled(true);
        }*/
    }
}
void SBRApp::onHandleCoatingParams()
{
    bool allFilled = !ui->coatThickIn->text().isEmpty() &&
        !ui->coatMuR->text().isEmpty() &&
        !ui->coatMuI->text().isEmpty() &&
        !ui->coatEpsR->text().isEmpty() &&
        !ui->coatEpsI->text().isEmpty();
    ui->coatImport->setEnabled(allFilled);
}
void SBRApp::onAddCoating()
{
    QString path = "E:\\Files";
    QString filter = "STL Files (*.stl)";
    QString meshFileName = QFileDialog::getOpenFileName(this, "Load ASCII .stl model(For Coating)", path, filter);
    
    if (!meshFileName.isEmpty()) {
        bool ok1, ok2, ok3, ok4, ok5;
        Float coatD = ui->coatThickIn->text().toFloat(&ok1);
        Float muRealPart = ui->coatMuR->text().toFloat(&ok2);
        Float muImagePart = ui->coatMuI->text().toFloat(&ok3);
        Float epsRealPart = ui->coatEpsR->text().toFloat(&ok4);
        Float epsImagePart = -ui->coatEpsI->text().toFloat(&ok5);
        if (ok1 && ok2 && ok3 && ok4 && ok5)
        {
            ParamCoat< Float > paramTmp(std::complex<Float>(epsRealPart, epsImagePart),
                std::complex<Float>(muRealPart, muImagePart),
                coatD);
            coatingParams.push_back(paramTmp);
            coatingMeshVecs.push_back(meshFileName.toStdString());
            ui->runMsg->insertPlainText("\n#Select " + meshFileName + " as Coating Meterial.");
            ui->runMsg->insertPlainText("\nthickness: " + QString::number(coatD, 'f', 5));
            ui->runMsg->insertPlainText(" | mu: " + QString::number(paramTmp.mu_.real(), 'f', 5) + " + " +
                QString::number(paramTmp.mu_.imag(), 'f', 5) + "i");
            ui->runMsg->insertPlainText(" | epsilon: " + QString::number(paramTmp.epsilon_.real(), 'f', 5) + " - " +
                QString::number(-paramTmp.epsilon_.imag(), 'f', 5) + "i");
        }
        else
        {
            ui->runMsg->insertPlainText("\nCoating Meterial Input Invalid.Check and retry.");
        }
        loadGL(meshFileName);
        readyMesh = true;/*
        if (readyMesh && readyParams)
        {
            ui->runButton->setEnabled(true);
        }*/
    }
}
void SBRApp::onHandleDielectParams()
{
    bool allFilled = !ui->dieMuR->text().isEmpty() &&
        !ui->dieMuI->text().isEmpty() &&
        !ui->dieEpsR->text().isEmpty() &&
        !ui->dieEpsI->text().isEmpty();
    ui->dieleImport->setEnabled(allFilled);
}
void SBRApp::onAddDie()
{
    QString path = "E:\\Files";
    QString filter = "STL Files (*.stl)";
    QString meshFileName = QFileDialog::getOpenFileName(this, "Load ASCII .stl model(For Dielectic)", path, filter);
    
    if (!meshFileName.isEmpty()) {
        bool ok1, ok2, ok3, ok4;
        Float muRealPart = ui->dieMuR->text().toFloat(&ok1);
        Float muImagePart = ui->dieMuI->text().toFloat(&ok2);
        Float epsRealPart = ui->dieEpsR->text().toFloat(&ok3);
        Float epsImagePart = -ui->dieEpsI->text().toFloat(&ok4);
        if (ok1 && ok2 && ok3 && ok4)
        {
            Param< Float > paramTmp(std::complex<Float>(epsRealPart, epsImagePart),
                std::complex<Float>(muRealPart, muImagePart));
            dielectParams.push_back(paramTmp);
            dielectMeshVecs.push_back(meshFileName.toStdString());
            ui->runMsg->insertPlainText("\n#Select " + meshFileName + " as Dieletric.");
            ui->runMsg->insertPlainText("\nmu: " + QString::number(paramTmp.mu_.real(), 'f', 5) + " + " +
                QString::number(paramTmp.mu_.imag(), 'f', 5) + "i");
            ui->runMsg->insertPlainText(" | epsilon: " + QString::number(paramTmp.epsilon_.real(), 'f', 5) + " - " +
                QString::number(-paramTmp.epsilon_.imag(), 'f', 5) + "i");
        }
        else
        {
            ui->runMsg->insertPlainText("\nDieletric Meterial Input Invalid.Check and retry.");
        }
        loadGL(meshFileName);
        readyMesh = true;/*
        if (readyMesh && readyParams)
        {
            ui->runButton->setEnabled(true);
        }*/
    }
}

void SBRApp::onConfirmParams()
{
    bool ok1, ok2, ok3, ok4, ok5, ok6, ok7, ok8;
    thetaIn = ui->thetaIn->text().toFloat(&ok1);
    phiIn = ui->phiIn->text().toFloat(&ok2);
    polarIn = ui->inWavePol->text().toFloat(&ok3);
    polarObs = ui->obsPol->text().toFloat(&ok4);
    fH = ui->freqHigh->text().toFloat(&ok5);
    fL = ui->freqLow->text().toFloat(&ok6);
    rayPerLambda = ui->rayPerLambda->text().toFloat(&ok7);
    if (polObsCnt == 2)
    {
        polarObsX = ui->obsPolX->text().toFloat(&ok8);
    }
    else
    {
        ok8 = true;
    }
    
    if (ok1 && ok2 && ok3 && ok4 && ok5 && ok6 && ok7 && ok8) {
        thetaIn = thetaIn * pi_ / 180.0f;
        phiIn = phiIn * pi_ / 180.0f;
        polarIn = polarIn * pi_ / 180.0f;
        polarObs = polarObs * pi_ / 180.0f;
        fH = fH * 1E9;
        fL = fL * 1E9;
        if (polObsCnt == 2)
        {
            polarObsX = polarObsX * pi_ / 180.0f;
        }
        if (ui->enterInTrCheck->isChecked())
        {
            enterInTr = true;
        }
        else
        {
            enterInTr = false;
        }
        readyParams = true;
        ui->runMsg->insertPlainText("\nIncident Wave Parameters Are Ready.");
    }
    else
    {
        readyParams = false;
        ui->runMsg->insertPlainText("\n!ERROR: Invalid Incident Wave Parameters.");
    }/*
    if (readyMesh && readyParams)
    {
        ui->runButton->setEnabled(true);
    }
    else
    {
        ui->runButton->setEnabled(false);
    }*/
}

void SBRApp::addPol()
{
    ui->obsPolX->setVisible(true); ui->obsPolXDeg->setVisible(true); ui->rmPolX->setVisible(true);
    ui->addPolX->setVisible(false);
    polObsCnt = 2;
}
void SBRApp::rmPol()
{
    ui->obsPolX->setVisible(false); ui->obsPolXDeg->setVisible(false); ui->rmPolX->setVisible(false);
    ui->addPolX->setVisible(true);
    polObsCnt = 1;
}

void SBRApp::onImport()
{
    QString path = "E:\\Files\\STL";
    QString filter = "STL Files (*.stl)";
    QString meshFileName = QFileDialog::getOpenFileName(this, "Load ASCII .stl model", path, filter);
    if (!meshFileName.isEmpty()) {
        ui->runMsg->insertPlainText("\n#Import " + meshFileName);

        std::ifstream file(meshFileName.toStdString());
        if (file.is_open()) {
            stlsplit sp;
            std::ostringstream oss;
            oss << file.rdbuf(); // Read the entire file content into a string
            file.close();
            std::string fileContent = oss.str();
            try
            {
                sp.LoadAndWrite(fileContent);
            }
            catch (const std::exception& e)
            {
                ui->runMsg->insertPlainText("\nException:" + QString::fromStdString(e.what()));
            }
            for (int i = 0; i < sp.solids_.size(); i++)
            {
                solids.push_back(Solid(sp.solids_[i], sp.solidPaths_[i]));
                //loadGL(QString::fromStdString(sp.solidPaths_[i])); 
                QStandardItem* solidItem = new QStandardItem(QIcon(":/SBRApp/icon/PEC.png"), QString::fromStdString(sp.solids_[i]));
                solidItem->setData(MARK_ITEM, ROLE_MARK);
                solidItem->setData(MARK_ITEM_SOLID, ROLE_MARK_ITEM);
                /*m_PEC->setData(MARK_ITEM, ROLE_MARK);
                m_PEC->setData(MARK_ITEM_TYPE, ROLE_MARK_ITEM);*/
                solidFolder->appendRow(solidItem);
                //ui->treeView->setModel(model);
            }
            reLoadGL();
        }
        else {
            ui->runMsg->insertPlainText("\nUnable to open file: " + meshFileName);
        }
    }
}

void SBRApp::onRunBtnClicked()
{
    passValueToSBR();
    if (!readyParams || !readyMesh)
    {
        if (readyMesh == false && readyParams == true)
        {
            QMessageBox::information(this, "Information", "Please confirm you have imported the mesh file.");
        }
        if (readyMesh == true && readyParams == false)
        {
            QMessageBox::information(this, "Information", 
                                    "Please confirm that you have entered all the parameters and clicked 'confirm'.");
        }
        if (readyMesh == false && readyParams == false)
        {
            QMessageBox::information(this, "Information",
                "Please confirm that you have imported the mesh file and entered all the parameters.");
        }
        return;
    }
    ui->runButton->setEnabled(false);
    QThread* thread = new QThread;
    try
    {
        if (polObsCnt == 1)
        {
            Worker* worker = new Worker(metalMeshVecs, coatingMeshVecs, dielectMeshVecs,
                coatingParams, dielectParams,
                thetaIn, phiIn, polarIn,
                polarObs, rayPerLambda,
                fH, fL,
                enterInTr);
            worker->moveToThread(thread);
            connect(thread, &QThread::started, worker, &Worker::doWork);
            bool connected = connect(worker, &Worker::calculationCompleted, this, &SBRApp::handleResults);
            qDebug() << "Connection successful:" << connected;
            connect(worker, &Worker::calculationCompleted, worker, &Worker::deleteLater);
            connect(thread, &QThread::finished, thread, &QThread::deleteLater);

            thread->start();
        }
        else
        {
            Worker* worker = new Worker(metalMeshVecs, coatingMeshVecs, dielectMeshVecs,
                coatingParams, dielectParams,
                thetaIn, phiIn, polarIn,
                polarObs, polarObsX, rayPerLambda,
                fH, fL,
                enterInTr);
            worker->moveToThread(thread);
            connect(thread, &QThread::started, worker, &Worker::doWork);
            bool connected = connect(worker, &Worker::calculationCompleted, this, &SBRApp::handleResults);
            qDebug() << "Connection successful:" << connected;
            connect(worker, &Worker::calculationCompleted, worker, &Worker::deleteLater);
            connect(thread, &QThread::finished, thread, &QThread::deleteLater);

            thread->start();
        }
    }
    catch (const std::exception& e)
    {
        ui->runMsg->insertPlainText("\nException:" + QString::fromStdString(e.what()));
    }
    
    
}

void SBRApp::handleResults(const std::vector< Float >& result, const U32& sampleNums)
{
    //ui->runMsg->insertPlainText("\n"+result);
    ui->freqLine->setVisible(true);
    QSplineSeries* LineSeries = new QSplineSeries();
    Float deltaFreq = 50 * fH / sampleNums;
    U32 idxFreqL = std::ceil(fL / deltaFreq) - 1;
    U32 idxFreqH = std::ceil(fH / deltaFreq) - 1;
    for (U32 i = idxFreqL; i <= idxFreqH; i++)
    {
        LineSeries->append(i * deltaFreq, result[i]);
    }
    QChart* chart = new QChart();
    chart->addSeries(LineSeries);
    chart->legend()->hide();
    chart->createDefaultAxes();
    chart->setTheme(QChart::ChartThemeDark);

    ui->freqLine->setChart(chart);
    if (polObsCnt == 2)
    {
        int mid = result.size() / 2;
        ui->freqLineX->setVisible(true);
        QSplineSeries* LineSeriesX = new QSplineSeries();
        for (U32 i = idxFreqL; i <= idxFreqH; i++)
        {
            LineSeriesX->append(i * deltaFreq, result[i + mid]);
        }
        QChart* chartX = new QChart();
        chartX->addSeries(LineSeriesX);
        chartX->legend()->hide();
        chartX->createDefaultAxes();
        chartX->setTheme(QChart::ChartThemeDark);

        ui->freqLineX->setChart(chartX);
    }
    ui->runButton->setEnabled(true);
    ui->runButton->setChecked(false);
    ui->runMsg->insertPlainText("\nReturn.("+QDateTime::currentDateTime().toString("hh:mm:ss")+")"); 
}

void SBRApp::clear()
{

}
void SBRApp::onRemoveSolidItem()
{
    QModelIndex curIndex = ui->treeView->currentIndex();
    int solidItemRowIdx = curIndex.row();
    solidFolder->removeRow(solidItemRowIdx);
    ui->runMsg->insertPlainText("\n#Removed " + QString::fromStdString(solids[solidItemRowIdx].name_));
    std::vector<Solid>::iterator iter = solids.begin() + solidItemRowIdx;
    solids.erase(iter);
    reLoadGL();
}
void SBRApp::onRemoveTypeItem()
{
    QModelIndex curIndex = ui->treeView->currentIndex();
    int typeItemRowIdx = curIndex.row();
    if (typeItemRowIdx == 0)
    {
        QMessageBox::information(this, "Information", "PEC can't be removed.");
        return;
    }
    typeFolder->removeRow(typeItemRowIdx);
    ui->runMsg->insertPlainText("\n#Removed " + QString::fromStdString(types[typeItemRowIdx].name_));
    std::vector<MType>::iterator iter = types.begin() + typeItemRowIdx;
    types.erase(iter);
}
void SBRApp::onAddType()
{
    QModelIndex curIndex = ui->treeView->currentIndex();
    QVariant var = curIndex.data(ROLE_MARK);
    if (var.isValid())
    {
        if (MARK_FOLDER == var.toInt()) //新增材料
        {
            dialog.show();
        }

    }
}
void SBRApp::clearAllTypes()
{
    int rowMaxIdx = typeFolder->rowCount() - 1;
    for (int i = rowMaxIdx; i > 0; i--)
    {
        typeFolder->removeRow(i);
    }
    for (int i = 0; i < rowMaxIdx; i++)
    {
        types.pop_back();
    }
    QMessageBox::information(this, "Attention!", 
        "Note: If you have specified a material for an object and then delete that material, it will result in unknown errors and program crashes.");
}
void SBRApp::clearAllSolids()
{
    int rowMaxIdx = solidFolder->rowCount() - 1;
    for (int i = rowMaxIdx; i >= 0; i--)
    {
        solidFolder->removeRow(i);
    }
    solids.clear();
    reLoadGL();
}
inline QString SBRApp::eraseLast0FloatToQStr(const Float x)
{
    std::string stdStrX = std::to_string(x);
    stdStrX.erase(stdStrX.find_last_not_of('0') + 1, std::string::npos);
    return QString::fromStdString(stdStrX);
}
void SBRApp::onEditType()
{
    QModelIndex curIndex = ui->treeView->currentIndex();
    int typeItemRowIdx = curIndex.row();
    QVariant var = curIndex.data(ROLE_MARK);
    if (typeItemRowIdx == 0)
    {
        QMessageBox::information(this, "Information", "PEC can't be edited."); 
        return;
    }
    MType mt = types[typeItemRowIdx];
    QStringList data;
    data.append(QString::fromStdString(mt.name_));
    if (mt.type_ == SolidType::DIELE)
    {
        std::string muR = std::to_string(mt.dieParam_.mu_.real());
        muR.erase(muR.find_last_not_of('0') + 1, std::string::npos);
        data.append(QString::fromStdString(muR));

        std::string muI = std::to_string(mt.dieParam_.mu_.imag());
        muI.erase(muI.find_last_not_of('0') + 1, std::string::npos);
        data.append(QString::fromStdString(muI));

        std::string epsR = std::to_string(mt.dieParam_.epsilon_.real());
        epsR.erase(epsR.find_last_not_of('0') + 1, std::string::npos);
        data.append(QString::fromStdString(epsR));

        std::string epsI = std::to_string(mt.dieParam_.epsilon_.imag());
        epsI.erase(epsI.find_last_not_of('0') + 1, std::string::npos);
        data.append(QString::fromStdString(epsI));
    }
    else if (mt.type_ == SolidType::COAT)
    {
        std::string muR = std::to_string(mt.coatParam_.mu_.real());
        muR.erase(muR.find_last_not_of('0') + 1, std::string::npos);
        data.append(QString::fromStdString(muR));

        std::string muI = std::to_string(mt.coatParam_.mu_.imag());
        muI.erase(muI.find_last_not_of('0') + 1, std::string::npos);
        data.append(QString::fromStdString(muI));

        std::string epsR = std::to_string(mt.coatParam_.epsilon_.real());
        epsR.erase(epsR.find_last_not_of('0') + 1, std::string::npos);
        data.append(QString::fromStdString(epsR));

        std::string epsI = std::to_string(mt.coatParam_.epsilon_.imag());
        epsI.erase(epsI.find_last_not_of('0') + 1, std::string::npos);
        data.append(QString::fromStdString(epsI));

        std::string coatD = std::to_string(mt.coatParam_.coatD_);
        coatD.erase(coatD.find_last_not_of('0') + 1, std::string::npos);
        data.append(QString::fromStdString(coatD));
    }
    emit sendEditTypeToDialog(data);

    if (var.isValid())
    {
        if (MARK_ITEM == var.toInt()) //编辑材料
        {
            dialog.show();
        }

    }
}
void SBRApp::receiveNewTypeFromDialog(QStringList data)
{
    MType newType;
    if (data.size() == 5) //介质材料 name+4参数
    {
        newType.type_ = SolidType::DIELE;
        newType.name_ = data.at(0).toStdString();
        newType.dieParam_.mu_ = std::complex< Float >(data.at(1).toFloat(), data.at(2).toFloat());
        newType.dieParam_.epsilon_ = std::complex< Float >(data.at(3).toFloat(), data.at(4).toFloat());
        types.push_back(newType);

        QStandardItem* typeItem = new QStandardItem(QIcon(":/SBRApp/icon/Diele.png"), data.at(0));
        typeItem->setData(MARK_ITEM, ROLE_MARK);
        typeItem->setData(MARK_ITEM_TYPE, ROLE_MARK_ITEM);
        typeFolder->appendRow(typeItem);
    }
    else if (data.size() == 6) //涂层材料 name+5参数
    {
        newType.type_ = SolidType::COAT;
        newType.name_ = data.at(0).toStdString();
        newType.coatParam_.mu_ = std::complex< Float >(data.at(1).toFloat(), data.at(2).toFloat());
        newType.coatParam_.epsilon_ = std::complex< Float >(data.at(3).toFloat(), data.at(4).toFloat());
        newType.coatParam_.coatD_ = data.at(5).toFloat();
        types.push_back(newType);

        QStandardItem* typeItem = new QStandardItem(QIcon(":/SBRApp/icon/Coat.png"), data.at(0));
        typeItem->setData(MARK_ITEM, ROLE_MARK);
        typeItem->setData(MARK_ITEM_TYPE, ROLE_MARK_ITEM);
        typeFolder->appendRow(typeItem);
    }
}
void SBRApp::receiveEditTypeFromDialog(QStringList data)
{
    QModelIndex curIndex = ui->treeView->currentIndex();
    int typeItemRowIdx = curIndex.row();
    MType editType;
    if (data.size() == 5) //介质材料 name+4参数
    {
        editType.type_ = SolidType::DIELE;
        editType.name_ = data.at(0).toStdString();
        editType.dieParam_.mu_ = std::complex< Float >(data.at(1).toFloat(), data.at(2).toFloat());
        editType.dieParam_.epsilon_ = std::complex< Float >(data.at(3).toFloat(), data.at(4).toFloat());
        types[typeItemRowIdx] = editType;

        QStandardItem* typeItem = typeFolder->child(typeItemRowIdx,0);
        typeItem->setIcon(QIcon(":/SBRApp/icon/Diele.png"));
        typeItem->setText(data.at(0));
    }
    else if (data.size() == 6) //涂层材料 name+5参数
    {
        editType.type_ = SolidType::COAT;
        editType.name_ = data.at(0).toStdString();
        editType.coatParam_.mu_ = std::complex< Float >(data.at(1).toFloat(), data.at(2).toFloat());
        editType.coatParam_.epsilon_ = std::complex< Float >(data.at(3).toFloat(), data.at(4).toFloat());
        editType.coatParam_.coatD_ = data.at(5).toFloat();
        types[typeItemRowIdx] = editType;

        QStandardItem* typeItem = typeFolder->child(typeItemRowIdx, 0);
        typeItem->setIcon(QIcon(":/SBRApp/icon/Coat.png"));
        typeItem->setText(data.at(0));
    }

}

void SBRApp::updateSolidTypeList(const QPoint& pos)
{
    /*QAction* selectType = solidItemMenu->actionAt(pos);
    QModelIndex curIndex = ui->treeView->indexAt(pos); */
    QModelIndex curIndex = ui->treeView->currentIndex(); 
    QVariant var = curIndex.data(ROLE_MARK);
    if (curIndex.isValid() && MARK_ITEM == var.toInt()) // if (curIndex.parent() != ui->treeView->rootIndex()) 条件不足
    {
        QVariant varIn = curIndex.data(ROLE_MARK_ITEM);
        if (varIn.isValid() && MARK_ITEM_SOLID == varIn.toInt())
        {
            solidItemSubMenu->clear();
            for (int i = 0; i < types.size(); i++)
            {
                QIcon icon;
                if (types[i].type_ == SolidType::PEC)
                {
                    icon.addFile(QString(":/SBRApp/icon/PEC.png"));
                }
                else if (types[i].type_ == SolidType::COAT)
                {
                    icon.addFile(QString(":/SBRApp/icon/Coat.png"));
                }
                else if (types[i].type_ == SolidType::DIELE)
                {
                    icon.addFile(QString(":/SBRApp/icon/Diele.png"));
                }
                QAction* ac = new QAction(icon, QString::fromStdString(types[i].name_), this);
                connect(ac, SIGNAL(triggered()), mapper, SLOT(map()));
                mapper->setMapping(ac, i);
                solidItemSubMenu->addAction(ac);
            }
            //connect(solidItemSubMenu, &QAction::triggered(QAction *), this, &SBRApp::setSolidType);
            selectType->setMenu(solidItemSubMenu);
        }
    }
}

void SBRApp::setSolidType(int typeIdx)
{
    QModelIndex curIndex = ui->treeView->currentIndex();
    int solidItemRowIdx = curIndex.row();

    solids[solidItemRowIdx].type_ = types[typeIdx].type_;
    solids[solidItemRowIdx].typePointer = &types[typeIdx];

    QStandardItem* solidItem = solidFolder->child(solidItemRowIdx, 0);
    if (solids[solidItemRowIdx].type_ == SolidType::PEC)
    {
        solidItem->setIcon(QIcon(":/SBRApp/icon/PEC.png"));
    }
    else if (solids[solidItemRowIdx].type_ == SolidType::COAT)
    {
        solidItem->setIcon(QIcon(":/SBRApp/icon/Coat.png"));
        QString toolTip;
        toolTip.append("mu:");
        toolTip.append(eraseLast0FloatToQStr(solids[solidItemRowIdx].typePointer->coatParam_.mu_.real()));
        toolTip.append(" + ");
        toolTip.append(eraseLast0FloatToQStr(solids[solidItemRowIdx].typePointer->coatParam_.mu_.imag()));
        toolTip.append("i");
        toolTip.append("  epsilon:");
        toolTip.append(eraseLast0FloatToQStr(solids[solidItemRowIdx].typePointer->coatParam_.epsilon_.real()));
        toolTip.append(" + ");
        toolTip.append(eraseLast0FloatToQStr(solids[solidItemRowIdx].typePointer->coatParam_.epsilon_.imag()));
        toolTip.append("i");
        toolTip.append("  d:");
        toolTip.append(eraseLast0FloatToQStr(solids[solidItemRowIdx].typePointer->coatParam_.coatD_));
        solidItem->setToolTip(toolTip);
    }
    else if (solids[solidItemRowIdx].type_ == SolidType::DIELE)
    {
        solidItem->setIcon(QIcon(":/SBRApp/icon/Diele.png"));
        QString toolTip;
        toolTip.append("mu:");
        toolTip.append(eraseLast0FloatToQStr(solids[solidItemRowIdx].typePointer->coatParam_.mu_.real()));
        toolTip.append(" + ");
        toolTip.append(eraseLast0FloatToQStr(solids[solidItemRowIdx].typePointer->coatParam_.mu_.imag()));
        toolTip.append("i");
        toolTip.append("  epsilon:");
        toolTip.append(eraseLast0FloatToQStr(solids[solidItemRowIdx].typePointer->coatParam_.epsilon_.real()));
        toolTip.append(" + ");
        toolTip.append(eraseLast0FloatToQStr(solids[solidItemRowIdx].typePointer->coatParam_.epsilon_.imag()));
        toolTip.append("i");
        solidItem->setToolTip(toolTip);
    }
    
}
void SBRApp::passValueToSBR()
{
    coatingParams.clear();
    dielectParams.clear();

    metalMeshVecs.clear();
    coatingMeshVecs.clear();
    dielectMeshVecs.clear();

    QString solidsInfo("\n#Solids Information:");
    int solidsCnt = solids.size();
    try
    {
        for (int idx = 0; idx < solidsCnt; idx++)
        {
            readyMesh = true;
            solidsInfo.append("\n[ " + QString::fromStdString(std::to_string(idx + 1)) + " ]");
            solidsInfo.append(QString::fromStdString(solids[idx].name_));
            if (solids[idx].type_ == SolidType::PEC)
            {
                solidsInfo.append("  PEC");
                metalMeshVecs.push_back(solids[idx].path_);
            }
            else if (solids[idx].type_ == SolidType::COAT)
            {
                solidsInfo.append("  Coating");
                coatingMeshVecs.push_back(solids[idx].path_);
                coatingParams.push_back(solids[idx].typePointer->coatParam_);
            }
            else if (solids[idx].type_ == SolidType::DIELE)
            {
                solidsInfo.append("  Dielectric");
                dielectMeshVecs.push_back(solids[idx].path_);
                dielectParams.push_back(solids[idx].typePointer->dieParam_);
            }
        }
    }
    catch (const std::exception& e)
    {
        readyMesh = false;
        ui->runMsg->insertPlainText("\nThere seems to be some issues with the STL file and material parameters. Please check your input.");
        ui->runMsg->insertPlainText("\nException:" + QString::fromStdString(e.what()));
    }
    
    ui->runMsg->insertPlainText(solidsInfo);
}
QString SBRApp::printSolidsAndParams()
{
    return QString("NULL");
}
Solid::Solid()
{
}
Solid::Solid(const std::string& name, const std::string& path)
{
    type_ = SolidType::PEC;
    name_ = name;
    path_ = path;

}
MType::MType(const std::string& name, const Float muR, const Float muI, const Float epsR, const Float epsI)
{
    type_ = SolidType::DIELE;
    name_ = name;
    dieParam_.mu_ = std::complex<Float>(muR, muI);
    dieParam_.epsilon_ = std::complex<Float>(epsR, epsI);
}
MType::MType(const std::string& name, const Float muR, const Float muI, const Float epsR, const Float epsI, const Float coatD)
{
    type_ = SolidType::COAT;
    name_ = name;
    coatParam_.mu_ = std::complex<Float>(muR, muI);
    coatParam_.epsilon_ = std::complex<Float>(epsR, epsI);
    coatParam_.coatD_ = coatD;
}
Solid::~Solid()
{
}
MType::MType()
{
}
MType::~MType()
{
}
Float SBRApp::pi_ = 3.14159265359;
