import getopt
import sys
import prog_mk2

def main(argv):
    inputfile = ''
    outputfile = ''
    page_size = 32
    and_verify = False

    opts, args = getopt.getopt(argv, "i:o:", ["page-size=", "verify"])
    for opt, arg in opts:
        if opt == "-i":
            inputfile = arg
        elif opt == "-o":
            outputfile = arg
        elif opt == "--page-size":
            print("setting page size to " + str(arg))
            page_size = int(arg)
        elif opt == "--verify":
            and_verify = True
        else:
            assert False, "unhandled option"

    if inputfile == '' or outputfile == '':
        raise FileNotFoundError("Missing input or output file")

    # generate our object
    prog = prog_mk2.Prog(outputfile)

    # program
    prog.program_device(inputfile, page_size, release_reset=not and_verify)

    # verify if we feel like it
    if and_verify:
        prog_verified = prog.verify_prog(inputfile)
    else:
        prog_verified = False

    return prog_verified


if __name__ == "__main__":
    main(sys.argv[1:])
