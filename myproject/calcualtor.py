import tkinter as tk

# Create main window
root = tk.Tk()
root.title("Ultamite Device")

# Entry field
entry = tk.Entry(root, width=20, font=('Arial', 18), borderwidth=2, relief="solid")
entry.grid(row=0, column=0, columnspan=4, padx=10, pady=10)

# Functions
def button_click(value):
    entry.insert(tk.END, value)

def clear_entry():
    entry.delete(0, tk.END)

def calculate():
    try:
        result = eval(entry.get())
        entry.delete(0, tk.END)
        entry.insert(0, str(result))
    except:
        entry.delete(0, tk.END)
        entry.insert(0, "Error")

# Buttons
buttons = [
    ('7',1,0), ('8',1,1), ('9',1,2), ('/',1,3),
    ('4',2,0), ('5',2,1), ('6',2,2), ('*',2,3),
    ('1',3,0), ('2',3,1), ('3',3,2), ('-',3,3),
    ('0',4,0), ('.',4,1), ('=',4,2), ('+',4,3),
    ('C',5,0)
]

for (text, row, col) in buttons:
    if text == '=':
        action = calculate
    elif text == 'C':
        action = clear_entry
    else:
        action = lambda val=text: button_click(val)

    tk.Button(root, text=text, width=5, height=2, font=('Arial', 18),
              command=action).grid(row=row, column=col, padx=5, pady=5)

# Run the app
root.mainloop()