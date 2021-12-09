:: Genenate the preprocessing file
iccarm --preprocess=n %1\scripts\preprocess_icf.i -I %1\..\src\config\ -I %1\..\..\..\..\..\sdk\common_project_files  %1\scripts\DA1458x_preprocess_icf.c

:: Generate the IAR *.icf file
python %1\scripts\make_app_dependent_icf.py -i %1\scripts\preprocess_icf.i -o %1\da14585_586.icf

del %1\scripts\preprocess_icf.i
