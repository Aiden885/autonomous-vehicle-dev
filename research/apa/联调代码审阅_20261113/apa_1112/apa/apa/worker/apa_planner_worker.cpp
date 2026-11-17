#include "apa_planning_job.h"
#include <csignal>
#include <cstdlib>
#include <unistd.h>

int main(int argc, char** argv) {
  if (argc != 2 || getppid() != std::strtol(argv[1], nullptr, 10)) return 2;

  std::signal(SIGALRM, SIG_DFL);
  sigset_t unblocked;
  sigemptyset(&unblocked); sigaddset(&unblocked, SIGALRM);
  if (sigprocmask(SIG_UNBLOCK, &unblocked, nullptr) != 0) return 2;
  try { return pangu::modules::RunApaPlannerWorker(3); }
  catch (...) { return 2; }
}
