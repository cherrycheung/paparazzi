#include <glib.h>
#include <stdio.h>
#include <stdlib.h>

#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>



gboolean timeout_callback(gpointer data) {
  int foo = 42;
  IvySendMsg("ME HELLO_WORLD 1234 5678 %d", foo);
  return TRUE;
}

int main ( int argc, char** argv) {


  IvyInit ("Example2", "Example2 READY", NULL, NULL, NULL, NULL);
  IvyStart("127.255.255.255");


  return 0;
}

