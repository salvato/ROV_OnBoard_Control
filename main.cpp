#include "ROV_app.h"
#include <signal.h>
#include <unistd.h>

ROV_App* pApp = NULL;


void
catchUnixSignals(const std::vector<int>& quitSignals,
                 const std::vector<int>& ignoreSignals = std::vector<int>()) {

    auto handler = [](int sig) ->void {
        printf("\nQuitting the application (user request signal = %d).\n", sig);
        pApp->switchOff();
    };

    // all these signals will be ignored.
    for(int sig : ignoreSignals)
        signal(sig, SIG_IGN);

    // each of these signals calls the handler (quits the QCoreApplication).
    for(int sig : quitSignals)
        signal(sig, handler);
}


int
main(int argc, char *argv[]) {
  pApp = new ROV_App(argc, argv);
  catchUnixSignals({SIGQUIT, SIGINT, SIGTERM, SIGHUP});
  int iResult = pApp->exec();
  delete pApp;
  return iResult;
}

