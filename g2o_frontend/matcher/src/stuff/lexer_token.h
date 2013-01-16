#ifndef _LEXER_TOKEN_H_
#define _LEXER_TOKEN_H_

#include <string>

struct LexerTokenValue{
	char	    charValue;
	int	    intValue;
	double	    doubleValue;
	std::string  strValue;
};

struct LexerToken {
  enum Type{Invalid=-1, Char=1, Int=2, Double=3, String=4, ID=5};
	Type type() const {return _type;}
	const LexerTokenValue& value() const {return _value;}
	LexerToken (char c){
		_type=Char;
		_value.charValue=c;
	};
	LexerToken (const char* c){
		_type=ID;	
		_value.strValue=std::string(c);
	};
	LexerToken (const char* c1, int lenght){
		_type=String;
		_value.strValue=std::string(c1,c1+lenght);
	};
	LexerToken (int i){
		_type=Int;
		_value.intValue=i;
	};
	LexerToken (double d){
		_type=Double;
		_value.doubleValue=d;
	};

protected:		
	LexerTokenValue _value;
	Type _type;
};

#endif
