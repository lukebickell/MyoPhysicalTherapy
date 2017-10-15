#ifndef ENTRYWINDOW_H
#define ENTRYWINDOW_H

#include <QMainWindow>

namespace Ui {
class EntryWindow;
}

class EntryWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit EntryWindow(QWidget *parent = 0);
    ~EntryWindow();

private:
    Ui::EntryWindow *ui;
};

#endif // ENTRYWINDOW_H
