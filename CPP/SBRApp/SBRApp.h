#pragma once
#ifndef _SILENCE_AMP_DEPRECATION_WARNINGS
#define _SILENCE_AMP_DEPRECATION_WARNINGS
#endif // !_SILENCE_AMP_DEPRECATION_WARNINGS
#include <QtWidgets/QMainWindow>
#include <QtCharts>
QT_CHARTS_USE_NAMESPACE
#include "ui_SBRApp.h"
#include <iostream>
#include <string>
#include "Observation.hpp"
#include "AppGLWidget.h"
#include "stlsplit.h"
#include "TypeWidget.h"
//#include "StlTrigMeshFile.hpp"
//#include "TypeDef.hpp"
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
#include"Worker.h"
QT_BEGIN_NAMESPACE
namespace Ui { class SBRAppClass; };
QT_END_NAMESPACE

enum SolidType : int
{
    PEC = 1,
    COAT = 2,
    DIELE = 4
};
class MType
{
public:
    SolidType type_;
    std::string name_;
    union
    {
        Param< Float > dieParam_;
        ParamCoat< Float > coatParam_;
    };
public:
    MType();
    MType(const std::string& name, const Float muR, const Float muI, const Float epsR, const Float epsI);
    MType(const std::string& name, const Float muR, const Float muI, const Float epsR, const Float epsI, const Float coatD);
    ~MType();

private:

};
class Solid
{
public:
    SolidType type_;
    std::string name_;
    std::string path_;
    MType* typePointer;
public:
    Solid();
    Solid(const std::string& name, const std::string& path);
    ~Solid();

private:

};
class SBRApp : public QMainWindow
{
    Q_OBJECT

public:
    SBRApp(QWidget *parent = nullptr);
    ~SBRApp();
    inline QString eraseLast0FloatToQStr(const Float);
public:
    TypeWidget dialog;
    QStandardItemModel* model;
    QStandardItem* solidFolder;
    QStandardItem* typeFolder;

    QMenu* solidFolderMenu;
    QMenu* solidItemMenu;
    QMenu* typeFolderMenu;
    QMenu* typeItemMenu;

    QMenu* solidItemSubMenu;;
    QAction* selectType;

    QSignalMapper* mapper;
public:
    void passValueToSBR();
    QString printSolidsAndParams();

private slots:
    void initTreeView();
    void onAddMetal();
    void onAddCoating();
    void onAddDie();

    void reLoadGL();
    void loadGL(const QString&);
    void onHandleCoatingParams();
    void onHandleDielectParams();
    void onConfirmParams();
    void onImport();
    void onRunBtnClicked();
    void handleResults(const std::vector< Float >&, const U32&);
    void clear();

    void addPol();
    void rmPol();

    void onTreeMenu(const QPoint& pos);
    void onRemoveSolidItem();
    void onRemoveTypeItem();
    void onAddType();
    void onEditType();

    void clearAllSolids();
    void clearAllTypes();

    void receiveNewTypeFromDialog(QStringList data);
    void receiveEditTypeFromDialog(QStringList data);

    void updateSolidTypeList(const QPoint& pos);

    void setSolidType(int idx);
signals:
    void sendEditTypeToDialog(QStringList data);
    
private:
    Ui::SBRAppClass *ui;
    std::vector<Solid> solids;
    std::vector<MType> types;
    std::vector< ParamCoat< Float > > coatingParams;
    std::vector< Param< Float > > dielectParams;

    std::vector< std::string > metalMeshVecs;
    std::vector< std::string > coatingMeshVecs;
    std::vector< std::string > dielectMeshVecs;

    bool enterInTr;
    Float thetaIn;
    Float phiIn;
    Float polarIn;
    Float polarObs;
    Float polarObsX;
    U32 polObsCnt;

    Float fH;
    Float fL;

    Float rayPerLambda;

    bool readyParams;
    bool readyMesh;
private:
    AppGLWidget* glView;
    Model m_;

public:
    static Float pi_;
};
