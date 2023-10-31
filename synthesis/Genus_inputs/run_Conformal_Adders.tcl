# Script to run Conformal to perform the Logig Equivalece Check
source /CMC/scripts/cadence.confrml21.10.100.csh
 
# LEC of RTL to Intermediate
#lec -XL -do ./outputs_Oct26-04:22:58/rtl2intermediate.lec.do

# LEC of Intermediate to Optimized
#lec -XL -do ./outputs_Oct26-04:22:58/intermediate2final.lec.do


# LEC of RTL to Optimized
lec -XL -do ./outputs_Oct26-04:22:58/rtl2final.lec.do

