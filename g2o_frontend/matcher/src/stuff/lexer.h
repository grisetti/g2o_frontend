#ifndef _SIMPLE_LEXER_H_
#define _SIMPLE_LEXER_H_

#include <iostream>
#include "lexer_token.h"
#include <FlexLexer.h>

struct Lexer: public yyFlexLexer{
  Lexer(std::istream* is=&std::cin, std::ostream* os=&std::cout);
  ~Lexer();
  bool nextToken();
  const LexerToken* currentToken() const ;
protected:
  LexerToken* _currentToken;
};

#endif
