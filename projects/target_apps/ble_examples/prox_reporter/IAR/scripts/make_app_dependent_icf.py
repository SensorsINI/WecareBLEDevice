import os
import sys, getopt
import string
import datetime
import shutil
start_tok = "/*This is an automatic generated section"
end_tok = "\n\r/*End of section */\n\r"

def load_i(in_file):
    out = start_tok + " " + datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S") + " */\n\r"
    with open(in_file) as file:
        for line in file:
            if "  int" in line:
                line = line.upper()
                line = line.replace('  INT','define symbol')
                out += line
    out = out + end_tok
    file.close()
    return out

def remove_autogen_block(fin):
    out_str = ""
    remove = False
    icf = open(fin, "r")

    for line in icf:
    #Slices to remove \n\r.
        if start_tok in line:
            remove = True
        elif end_tok[2:-2] in line:
            remove = False
            continue
        if not remove:
            out_str += line
    icf.close()
    return out_str

def main(argv):
    in_file = ''
    out_file = ''
    try:
        opts, args = getopt.getopt(argv,"hi:o:",["ifile=","ofile="])
    except getopt.GetoptError:
        print ('make_app_dependent.py -i <inputfile> -o <outputfile>')
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print( 'test.py -i inputfile -o outputfile')
            sys.exit()
        elif opt in ("-i", "--ifile"):
            in_file = arg
        elif opt in ("-o", "--ofile"):
            out_file = arg

    gen = load_i(in_file)
    icf = remove_autogen_block(out_file)
    if icf and gen:
        fout = open(out_file, "w")
        fout.write(gen + icf)
        fout.close()
    else:
        print("Error: icf generation failed.")

if __name__ == "__main__":
    main(sys.argv[1:])

