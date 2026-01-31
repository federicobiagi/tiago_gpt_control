# coding=utf-8
import os
from pathlib import Path
import argparse
import re
from tiago_arm_control_clean import TiagoControl
import rclpy
from rclpy.node import Node
import openai
import json


class GPTManager(Node):
    def __init__(self, api_key: str, model: str = "gpt-5-mini", with_tools: bool = True):
        super().__init__("gpt_manager")

        if not api_key:
            raise RuntimeError("OPENAI_API_KEY not found and --api_key not provided")

        openai.api_key = api_key
        self.model = model
        self.with_tools = with_tools
    
        script_dir = Path(__file__).parent.resolve()

        if self.with_tools:
            sysprompt_path = script_dir / "system_prompt_clean.txt"
            sysprompt = sysprompt_path.read_text(encoding="utf-8")
            self.chat_history = [{"role": "system", "content": sysprompt}]
        else:
            sysprompt_path = script_dir / "system_prompt_no_tools.txt"
            sysprompt = sysprompt_path.read_text(encoding="utf-8")
            self.chat_history = [{"role": "system", "content": sysprompt}]

        self.tiago = TiagoControl()   
        
        tools_path = script_dir / "tools.json"
        tools_content = tools_path.read_text(encoding="utf-8")
        self.tools = json.loads(tools_content)




    def ask(self, question: str) -> str:
        """Send a question to the model and return the answer."""
        self.chat_history.append({"role": "user", "content": question})

        if self.with_tools:
            response = openai.chat.completions.create(
                model=self.model,
                messages=self.chat_history,
                tools=self.tools,
                tool_choice="auto",
            )
            msg = response.choices[0].message
            if msg.tool_calls:
                for tool_call in msg.tool_calls:
                    tool_name = tool_call.function.name
                    tool_args = json.loads(tool_call.function.arguments)
                    answer = "Esecuzione del tool {} con argomenti {}. Risultato: ".format(tool_name, tool_args)
                    self.get_logger().info(f"Chiamata tool: {tool_name} con argomenti {tool_args}")
                    if tool_name == "move_distance":
                        self.tiago.move_distance(**tool_args)
                        answer += "Movimento completato di {} metri.".format(tool_args['distance'])
                    elif tool_name == "rotate_angle":
                        self.tiago.rotate_angle(**tool_args)
                        answer += "Rotazione completata di {} radianti.".format(tool_args['angle'])
                    elif tool_name == "move_arm":
                        tool_args['joint_positions'] = [float(x) for x in tool_args['joint_positions']]
                        self.tiago.move_arm(**tool_args)
                        answer += "Braccio mosso ai joint angle {}.".format(tool_args['joint_positions'])
                    elif tool_name == "grasp_object_by_name_front":
                        self.tiago.grasp_object_by_name_front(**tool_args)
                        answer += "Afferrato l'oggetto {}.".format(tool_args['object_name'])
                    elif tool_name == "open_gripper":
                        self.tiago.open_gripper()
                        answer += "Gripper aperto."
                    elif tool_name == "move_gripper":
                        self.tiago.move_gripper(**tool_args)
                        answer += "Movimento del gripper completato."
                    elif tool_name == "detach_object":
                        self.tiago.detach_object(**tool_args)
                        answer += "Oggetto rilasciato."
                    self.chat_history.append({"role": "assistant", "content": answer})

                    
            else:
                answer = msg.content
                self.chat_history.append({"role": "assistant", "content": answer})
            return answer
        else:
            response = openai.chat.completions.create(
                model=self.model,
                messages=self.chat_history,
            )
            answer = response.choices[0].message.content
            self.extract_python_code(answer)
            f = open("./code.txt", "r")
            linesofcode = f.readlines()
            for code in linesofcode:  #esegui la lista di metodi che GPT ha prodotto
                #print("Codice da eseguire: " + code)
                code = code.replace('\n','')
                if self.check_code_integrity(code):
                    print("Step di codice da eseguire: ")
                    print(code)
                    exec('self.tiago.' + code.replace('\\',""))
            self.chat_history.append({"role": "assistant", "content": answer})
            return answer
        
    def check_code_integrity(self, code: str) -> bool:
        forbidden_keywords = ["import", "open", "exec", "eval", "__", "os.", "sys.", "subprocess", "shutil"]
        for keyword in forbidden_keywords:
            if keyword in code:
                self.get_logger().warning(f"Codice non sicuro rilevato: {keyword}")
                return False
        return True
    
    def extract_python_code(self,content):
        code_block_regex = re.compile(r"```(.*?)```",re.DOTALL)  #prende il pezzo di codice che è specificato in "assistant" "content" tra 3 apici inversi
        code_blocks = code_block_regex.findall(content) #trova quanto specificato secondo le regole definite prima con re.compile, cioè estrae il testo compreso tra ```
        if code_blocks:
            full_code = "\n".join(code_blocks) #aggrega le stringhe separandole con un 'a capo'

            if full_code.startswith("python"):
                full_code = full_code[7:]  #prende il codice dalla parola 'python' in poi, non compresa
                full_code = full_code.replace("\\n", "\n")
                open('./code.txt','w').write(full_code)

            return full_code.rstrip()
        else:
            return None


    def run(self):
        print("Nodo GPT avviato. Comand TIAGO") 
        print("Scrivi 'exit' o 'quit' per fermare.\n")

        try:
            while rclpy.ok():
                try:
                    user_input = input("User: ").strip()
                except EOFError:
                    break

                if not user_input:
                    continue

                if user_input.lower() in ("exit", "quit"):
                    break

                answer = self.ask(user_input)
                if answer and not 'python' in answer:
                    print(f"Gpt Bot: {answer}\n")

        except KeyboardInterrupt:
            pass


def main():
    rclpy.init()

    parser = argparse.ArgumentParser()
    parser.add_argument("--api_key", type=str, default=os.getenv("OPENAI_Personal_Key"), help="OpenAI API key")
    parser.add_argument("--model", type=str, default="gpt-5", help="OpenAI model to use")
    parser.add_argument("--tools", type=bool, default=False, help="Enable tool usage" )
    args = parser.parse_args()

    node = GPTManager(api_key=args.api_key, model=args.model, with_tools=args.tools)

    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()




