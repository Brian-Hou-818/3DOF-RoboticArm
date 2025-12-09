def morseToLetter(morse):
    morse_dict = {
    '._': 'A',
    '_...': 'B',
    '_._.': 'C',
    '_..': 'D',
    '.': 'E',
    '.._.': 'F',
    '__.': 'G',
    '....': 'H',
    '..': 'I',
    '.___': 'J',
    '_._': 'K',
    '._..': 'L',
    '__': 'M',
    '_.': 'N',
    '___': 'O',
    '.__.': 'P',
    '__._': 'Q',
    '._.': 'R',
    '...': 'S',
    '_': 'T',
    '.._': 'U',
    '..._': 'V',
    '.__': 'W',
    '_.._': 'X',
    '_.__': 'Y',
    '__..': 'Z',
    '_____': '0',
    '.____': '1',
    '..___': '2',
    '...__': '3',
    '...._': '4',
    '.....': '5',
    '_....': '6',
    '__...': '7',
    '___..': '8',
    '____.': '9',}

    return morse_dict[morse]

def location(letter):
    location_dict = {
        'A': [3, 4, -15],
        'B': [7, 8, -15]}
    return location_dict[letter]

inputLetter = str(input("Morse Code: "))
    # move_X = float(input("X: "))
    # move_Y = float(input("Y: "))
    # move_Z = float(input("Z: ")) + 3

move_X = location(morseToLetter(inputLetter))[0]
move_Y = location(morseToLetter(inputLetter))[1]
move_Z = location(morseToLetter(inputLetter))[2]

print(move_X, move_Y)