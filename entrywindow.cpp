#include "entrywindow.h"
#include "ui_entrywindow.h"

EntryWindow::EntryWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::EntryWindow)
{
    ui->setupUi(this);
}

EntryWindow::~EntryWindow()
{
    delete ui;
}
