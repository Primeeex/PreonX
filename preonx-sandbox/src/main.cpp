#include "application.hpp"
#include <cstdio>

int main(int argc, char* argv[]) {
    Application app;
    app.startup(argc, argv);
    if (app.state().running) {
        app.run();
    }
    app.shutdown();
    return 0;
}
