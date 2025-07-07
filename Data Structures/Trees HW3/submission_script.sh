#!\bin\bash

# This script does not work inside the container (because zip is not installed), 
# but you can run it outside the container
 
# DO NOT MODIFY THIS FILE

RED='\033[0;31m'
NC='\033[0m' # No Color

# if submission folder does not exist, create it
if [ ! -d "submission" ]; then
  mkdir submission
fi

# Copy tree and interface files, if copy fails, echo error message
for file in src/tree.c include/tree.h src/interface.c include/interface.h src/game_bot.c include/game_bot.h; do
  if ! cp "$file" "submission/$(basename "$file")"; then
    echo "${RED}ERROR: ${file} file not found${NC}"
  fi
done

zip -r submission.zip submission
rm -r submission