from openai import OpenAI 

def ask_llm(query):
    """
    Send a query to chatgpt o1
    
    Args:
        query (str): The problem or question to solve
        
    Returns:
        str: The model's response
    """
    client = OpenAI(api_key="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")

    prompt = """
    You are a mathematical reasoning assistant specialized in solving the Tower of Hanoi puzzle using a tagged move format.

    # Goal:
    - The objective of the Tower of Hanoi puzzle is to move the entire stack of disks from the source rod to the destination rod.
    - There are three rods A, B, and C.
    - Disks are numbered from 1 (smallest) to N (largest).

    # Rules:
    - Only one disk may be moved at a time.
    - Each move consists of taking the upper disk from one of the stacks and placing it on top of another stack or on an empty rod.
    - A larger disk may not be placed on top of a smaller one.
    - If a starting state is against the rules then you can still move the upper disk away or place a disk ontop if it is smaller.

    # Move Tag Format:
    Each move is formatted as:
        MD<disk_number><source_peg><destination_peg>

    For example:
        MD1AC  → Move disk 1 from rod A to rod C
        MD3CB  → Move disk 3 from rod C to rod B

    # Your Task
    Generate the list of moves (in tagged format) to solve the Tower of Hanoi puzzle from the given start to the target state. Make sure all moves obey the rules and are valid. Label the moves with step numbers.
    Respond only with the move list in a format like this:
    1. MD1AC  
    2. MD2AB  
    3. MD1CB  

    """

    print(prompt+query)

    try:
        response = client.responses.create(
            model="o1",
            reasoning={"effort":"medium"},
            input=[
                {"role": "user", "content": f"{prompt + query}"}
            ],
            store=False,
            max_output_tokens=6000,
        )

        if response.status == "incomplete" and response.incomplete_details.reason == "max_output_tokens":
            print("Ran out of tokens")
            if response.output_text:
                return response
            else: 
                print("Ran out of tokens during reasoning")

        return response
    
    except Exception as e:
        return f"Error: {e}"

# Example usage
if __name__ == "__main__":

    context = """
    You are a mathematical reasoning assistant specialized in solving the Tower of Hanoi puzzle using a tagged move format.

    # Goal:
    - The objective of the Tower of Hanoi puzzle is to move the entire stack of disks from the source rod to the destination rod.
    - There are three rods A, B, and C.
    - Disks are numbered from 1 (smallest) to N (largest).

    # Rules:
    - Only one disk may be moved at a time.
    - Each move consists of taking the upper disk from one of the stacks and placing it on top of another stack or on an empty rod.
    - A larger disk may not be placed on top of a smaller one.
    - If a starting state is against the rules then you can still move the upper disk away to an empty rod or place a disk ontop if it is smaller.

    # Move Tag Format:
    Each move is formatted as:
        MD<disk_number><source_peg><destination_peg>

    For example:
        MD1AC  → Move disk 1 from rod A to rod C
        MD3CB  → Move disk 3 from rod C to rod B

    # Your Task
    Generate the list of moves (in tagged format) to solve the Tower of Hanoi puzzle from the given start to the target state. Make sure all moves obey the rules and are valid. Label the moves with step numbers.
    Respond only with the move list in a format like this:
    1. MD1AC  
    2. MD2AB  
    3. MD1CB  

    ### Current Problem:
    - Number of Disks: 3
    - Starting State (bottom disk first, top disk last):
        Rod A: [3, 1, 2]
        Rod B: []
        Rod C: []
    - Target State:
        Rod A: []
        Rod B: [3, 2, 1]
        Rod C: []
    
    """

    print(f"Query: {context}")
    
    solution = ask_llm(context)
    print(f"Solution:\n{solution}")