#include <QtWidgets/QApplication>

#include "MyWindow.h"
#define DEBUG

#ifdef DEBUG
#include <windows.h>
#include <stdio.h>
#endif

int main(int argc, char** argv) {
#ifdef DEBUG
	FreeConsole();

	// create a separate new console window
	AllocConsole();

	// attach the new console to this application's process
	AttachConsole(GetCurrentProcessId());

	// reopen the std I/O streams to redirect I/O to the new console
	freopen("CON", "w", stdout);
	freopen("CON", "w", stderr);
	freopen("CON", "r", stdin);
#endif
	QApplication app(argc, argv);
	MyWindow window(&app);
	window.show();
	return app.exec();
}
