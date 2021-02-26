#include <QtWidgets>
#include <QTest>

#include "../gui/editor.h"

class TestGui : public QObject
{
  Q_OBJECT

private:
  Editor* editor = nullptr;

private slots:
  void initTestCase()
  {
    printf("initTestCase()\n");
    editor = new Editor();
  }
  void testGui()
  {
    //QCOMPARE("a", "b");
  }
  void cleanupTestCase()
  {
    printf("cleanupTestCase()\n");
    delete editor;
  }
};

QTEST_MAIN(TestGui)
#include "test_gui.moc"
