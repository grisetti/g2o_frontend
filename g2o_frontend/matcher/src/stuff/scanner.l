%option c++
%option noyywrap

%{
#include<sstream>
#include "lexer_token.h"
using namespace std;
%}

DIGIT              [0-9]
LITERAL            [a-zA-Z_]
INTEGER            ("+"|"-")?{DIGIT}+
DOUBLE              {INTEGER}"."{DIGIT}*?(("e"|"E"){INTEGER})?
ID                 {LITERAL}({DIGIT}|{LITERAL})*
NUMBER		   {FLOAT}|{INTEGER}
STRING		   \"[^\n\"]*\"
COMMENT		   "//"[^\n]*[\n]
%%

{INTEGER} 	{
			return LexerToken::Int;
		}

{DOUBLE} 		{
			//std::cerr <<"FLOAT: "<< atof(yytext) <<std::endl;
			return LexerToken::Double;
		}	

{STRING}	{	
			//std::cerr << "STRING";
			return LexerToken::String;
	        }

{ID} 		{
			//std::cerr <<"ID: "<< yytext <<std::endl;
			return LexerToken::ID;
		}

{COMMENT}|[\x20]* {
			//std::cerr <<"COMMENT: "<< yytext <<std::endl;
			return LexerToken::Invalid;
		}

.		{
			//std::cerr <<"CHAR: "<< yytext <<std::endl;
			return LexerToken::Char;
		}



%%