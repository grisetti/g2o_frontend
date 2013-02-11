#!/bin/bash
git mv $1 $2
sed -i s/$1/$2/g *