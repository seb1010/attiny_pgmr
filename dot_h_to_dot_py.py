import sys

def main(argv):
    output_text = ""
    with open(argv[0], 'r') as fp:
        line = fp.readline()
        i = 1
        while line != "":
          output_text += reformat_line(line, i);
          line = fp.readline()
          i+=1

    new_filename = argv[0][0:-2] + ".py"
    with open(new_filename, 'w') as fp:
        fp.write(output_text)

def reformat_line(line: str, line_num: int):

    # remove leading whitespace
    i = 0;
    while(line[i] == " " or line[i] == "\t"):
        i+=1
    newline = line[i:]

    # remove comments    
    if len(newline) == 1 or newline[0:2] == "//":
        ret_line = ""
    elif newline[0:7] != "#define":
        ret_line = ""
        raise ValueError(f"INVALID LINE: {line_num}")
    else:
        i = 8
        while newline[i] != " " and i < len(newline):
            if i == len(newline) - 1:
                raise ValueError(f"INVALID LINE: {line_num}")
            i+=1

        # remove same line comments
        j = i
        while j < len(newline) and newline[j] != "/":
            j+=1

        # remove all newlines
        if newline[j - 1] == "\n":
          j -= 1
            
        ret_line = newline[8:(i+1)] + "=" + newline[i:j] + "\n"

    return ret_line


if __name__ == "__main__":
    main(sys.argv[1:])
