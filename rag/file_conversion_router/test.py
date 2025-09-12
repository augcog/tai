a = 'Estimation of $( \\'
a=a.replace('\\', '').strip()
b = ' Estimation of $( \mathrm { R } , \mathrm { T } )$'
b=b.replace('\\', '').strip()

print(a)
print(b)
print(a in b)