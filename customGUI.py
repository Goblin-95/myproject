import tkinter as tk

root = tk.Tk()
root.title("String editing") # creates blank canvas and names it

def removeVowels():
    text = entry.get()
    vowels = "aeiouAEIOU"
    result = ""

    for char in text:
        if char not in vowels:
            result += char

    entry.delete(0, tk.END)
    entry.insert(0, str(result))

def reverseString():
    text = entry.get()
    reverse = ""

    for char in text:
        reverse = char + reverse

    entry.delete(0, tk.END)
    entry.insert(0, str(reverse))

def capitalize():
    text = entry.get()
    capitalize = ""

    capitalize = text.upper()

    entry.delete(0, tk.END)
    entry.insert(0, str(capitalize))
    

    

entry = tk.Entry(root, width = 20, font = ('Arial', 18), borderwidth = 2, relief = "solid")
entry.pack(padx = 100, pady = 20)

button1 = tk.Button(root, text = "Remove vowels", command = removeVowels) # creates button functionality
button1.pack(padx = 100, pady = 10)  # adds button to screen

button2 = tk.Button(root, text = "Reverse text", command = reverseString) # creates button functionality
button2.pack(padx = 100, pady = 5)  # adds button to screen

button3 = tk.Button(root, text = "Capitalize/Decapitalize", command = capitalize) # creates button functionality
button3.pack(padx = 100, pady = 5)  # adds button to screen

root.mainloop()