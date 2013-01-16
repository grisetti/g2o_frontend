#include <iostream>
#include "lexer.h"

using namespace std;


int main (int argc, char** argv) {
  Lexer* lexer = new Lexer(&std::cin);
  while (lexer->nextToken()) {
    const LexerToken* token=lexer->currentToken();
    if (token){
      cerr << "Token type=" << token->type() << " ";
      switch (token->type()){
      case LexerToken::Char: 
	cerr << token->value().charValue;
	break;
      case LexerToken::Int: 
	cerr << token->value().intValue;
	break;
      case LexerToken::Double: 
	cerr << token->value().doubleValue;
	break;
      case LexerToken::ID: 
	cerr << token->value().strValue;
	break;
      case LexerToken::String: 
	cerr << token->value().strValue;
	break;
      default: cerr << "Unknown";
	break;
      }
    }
    cerr << "." << endl;
  }
  return 0;
}
