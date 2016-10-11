
#include "MyForm.h"
#include "include/utwsapi.h"

rrd1 rrd;
Socket sock;

using namespace System;
using namespace System::Windows::Forms;

[STAThreadAttribute]
int main(array<String^>^ args) {
	Application::EnableVisualStyles();
	Application::SetCompatibleTextRenderingDefault(false);
	// gcnew [1]で付けたプロジェクト名::[2]で付けたForm名()
	Application::Run(gcnew WHS1_test::MyForm());
	return 0;
}