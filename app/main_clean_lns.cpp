#include "lnssat/PublicResult.h"
int main(int argc, char** argv) {
    return lnssat::public_cli(argc,argv,static_cast<lnssat::SolveFunction>(&LNS));
}
