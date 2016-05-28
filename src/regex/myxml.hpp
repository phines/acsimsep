#ifndef MYXML_HPP
#define MYXML_HPP

#include <stdio.h>
#include <libxml/parser.h>
#include <libxml/tree.h>

#ifndef LIBXML_TREE_ENABLED

int main(void) {
    fprintf(stderr, "Tree support not compiled in\n");
    exit(1);
}

#endif /* end LIBXML_TREE_ENABLED */
#endif
