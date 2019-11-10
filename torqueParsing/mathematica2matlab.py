
# torquesPath = '/home/matt/Grad School/Robotics/project/matlabCode/torques_mathematica'
#
# torquesText = []
# with open(torquesPath, 'r') as f:
#     for line in f:
#         torquesText.append( line.replace('\n','') )
#
# print(torquesText[4])


torquesPath = '/home/matt/Grad School/Robotics/project/matlabCode/torques_mathematica_expanded'

# need to iterate through and remove the line breaks, joining everything back into a single line so we can resolve
# the additional parameters that we need to
torquesTextAll = []
with open(torquesPath, 'r') as f:
    for line in f:
        line = line.replace('\n', '')
        if line == '  \\':
            continue
        elif line[-3:] == '...':
            line = line[:-3]
        elif line[-1] == '\\':
            line = line[:-1]
        line = line.strip() # make sure there aren't any spaces on either side
        torquesTextAll.append( line )

        # if len(torquesTextAll) > 200:
        #     break

torquesText = ''.join(torquesTextAll)


# alright so now what do we still need to fix?
# - Subscript(var, 1) to var1
# - \\[Theta] to theta
# - (t) to just delete?
# - (t).*Derivative(1) to _dot - THIS NEEDS TO RUN BEFORE THE replace('t','')!!
# - (t).*Derivative(2) to _ddot
# - also our initial parser is interpreting I_xx7 as the imaginary i and replacing with sqrt(-1) ... lolwat

torquesText = torquesText.replace('\\[Theta]','theta')
torquesText = torquesText.replace('sqrt(-1)','I') # fix the inertia parsing
# torquesText = torquesText.replace('(t).*Derivative(1)','_dot') #  hopefully just add on to the previous var name
# torquesText = torquesText.replace('(t).*Derivative(2)', '_ddot') # hopefully just add on to the previous var name


# so also the derivatives actually apply to the value AFTER, not before ... so need to parse these differently as well





# then need to actually iterate through to resolve the Subscript() stuff
key = 'Subscript('
while key in torquesText:
    ind = torquesText.find(key) # we'll keep deleting them as we go, so should be safe to keep resolving the first one
    commaInd = torquesText.find(',', ind)
    endInd = torquesText.find(')', commaInd)
    val1 = torquesText[ind+len(key) : commaInd]
    val2 = torquesText[commaInd+1 : endInd]

    if val2.isdigit():
        replaceStr = val1 + '(' + val2 + ')' # actually lets treat this as vector indices
    else:
        # this is probably the inertial subscripts?
        if len(val2) == 3 and val2[0] in ['x','y','z'] and val2[-1].isdigit():
            # should be an inertia
            # I guess track the inertias as a 3d array?
            inertiaInd = val2[-1]
            if val2[0:2] == 'xx':
                replaceStr = val1 + '(' + inertiaInd + ',1,1)'
            elif val2[0:2] == 'yy':
                replaceStr = val1 + '(' + inertiaInd + ',2,2)'
            elif val2[0:2] == 'zz':
                replaceStr = val1 + '(' + inertiaInd + ',3,3)'
        else:
            raise NotImplementedError()

    totalStr = torquesText[ind : endInd+1]
    torquesText = torquesText.replace(totalStr, replaceStr)


# lets try resolving the subscripts before derivatives?
torquesText = torquesText.replace('(t)', '') # delete these first, these come after the derivative statement
key = 'Derivative('
while key in torquesText:
    ind = torquesText.find(key)
    order = int(torquesText[ind + len(key)])
    termStart = ind + len(key) + 3

    # then just find the next closing parenthesis since subscript should be resolved?
    # well actually we also have array indexing... so lets take the min of the next left or right paren
    leftParen = torquesText.find('(', termStart)
    rightParen = torquesText.find(')',termStart)

    if leftParen < rightParen:
        # assume we have indexing with the right paren being part of the term
        index = torquesText[leftParen+1 : rightParen]
        replaceStr = torquesText[termStart : leftParen] + '_' + order*'d' + 'ot' + '(' + index + ')'
        rightParen += 1 # have an extra parenthesis in here

    else:
        # term = torquesText[termStart:termEnd]
        # replaceStr = term + '_' + order*'d' + 'ot'
        raise NotImplementedError()

    totalStr = torquesText[ind : rightParen+1] # end inclusive
    torquesText = torquesText.replace(totalStr, replaceStr)

torquesTextFull = 'torques = ' + torquesText
with open('torques.m', 'w') as f:
    f.write(torquesTextFull)


# we now want to try to split each term by it's derivative order for configuration space form
torques = torquesText[1:-2].split(';')

torque = torques[0] # simple one

# terms = torque.replace('-','+').split('+')

# need to also search through all the '-' and see if they're negative values vs subtraction between terms
# fortunately negative values should be surrounded in parenthesis
searchInd = 0
while '-' in torque[searchInd:]:
    ind = torque.find('-', searchInd)
    if torque[ind-1] == '(':
        searchInd = ind + 1 # is a negative, move on
        continue
    else:
        torque[ind] = '|' # is a separate term, replace it

torque = torque.replace('+','|')
terms = torque.split('|')

for term in terms:
    grav = term.count('g')
    o1 = term.count('_dot')
    o2 = term.count('_ddot')
    print(term)
    print(grav, o1, o2)



# TODO: we can split the terms based on the counts above, they seem to only ever have one of each in them
# though there are some terms that don't have any of the gs or derivatives - is this correct? what do we do with them?


print('')



