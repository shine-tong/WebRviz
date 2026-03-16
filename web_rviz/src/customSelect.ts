export interface CustomSelectController {
  refresh: () => void;
  dispose: () => void;
}

const customSelectControllers = new Map<HTMLSelectElement, CustomSelectController>();
let globalListenersAttached = false;

function closeAllCustomSelects(except?: HTMLSelectElement): void {
  for (const [select, controller] of customSelectControllers.entries()) {
    if (select === except) {
      continue;
    }
    const wrapper = select.parentElement;
    if (wrapper) {
      wrapper.classList.remove("open");
    }
  }
}

function attachGlobalListeners(): void {
  if (globalListenersAttached) {
    return;
  }
  globalListenersAttached = true;

  document.addEventListener("click", (event) => {
    const target = event.target as Node | null;
    for (const select of customSelectControllers.keys()) {
      const wrapper = select.parentElement;
      if (wrapper && target && wrapper.contains(target)) {
        return;
      }
    }
    closeAllCustomSelects();
  });

  document.addEventListener("keydown", (event) => {
    if (event.key === "Escape") {
      closeAllCustomSelects();
    }
  });
}

export function enhanceSelect(select: HTMLSelectElement): CustomSelectController {
  const existing = customSelectControllers.get(select);
  if (existing) {
    existing.refresh();
    return existing;
  }

  attachGlobalListeners();

  const parent = select.parentElement;
  if (!parent) {
    throw new Error("custom select requires a parent element");
  }

  const wrapper = document.createElement("div");
  wrapper.className = "custom-select";

  const trigger = document.createElement("button");
  trigger.type = "button";
  trigger.className = "custom-select-trigger";
  trigger.setAttribute("aria-haspopup", "listbox");

  const menu = document.createElement("div");
  menu.className = "custom-select-menu";
  menu.setAttribute("role", "listbox");

  parent.insertBefore(wrapper, select);
  wrapper.appendChild(select);
  wrapper.appendChild(trigger);
  wrapper.appendChild(menu);
  select.classList.add("custom-select-native");

  const render = (): void => {
    menu.innerHTML = "";

    const selectedOption = select.selectedOptions[0] ?? select.options[0] ?? null;
    trigger.textContent = selectedOption?.textContent ?? "";
    trigger.disabled = select.disabled || select.options.length === 0;
    trigger.setAttribute("aria-expanded", wrapper.classList.contains("open") ? "true" : "false");

    for (const option of Array.from(select.options)) {
      const optionButton = document.createElement("button");
      optionButton.type = "button";
      optionButton.className = "custom-select-option";
      optionButton.textContent = option.textContent ?? option.value;
      optionButton.disabled = option.disabled;
      optionButton.dataset.value = option.value;
      optionButton.classList.toggle("selected", option.value === select.value);
      optionButton.setAttribute("role", "option");
      optionButton.setAttribute("aria-selected", option.value === select.value ? "true" : "false");
      optionButton.addEventListener("click", () => {
        if (option.disabled) {
          return;
        }
        const changed = select.value !== option.value;
        select.value = option.value;
        wrapper.classList.remove("open");
        render();
        if (changed) {
          select.dispatchEvent(new Event("change", { bubbles: true }));
        }
      });
      menu.appendChild(optionButton);
    }
  };

  const observer = new MutationObserver(() => {
    render();
  });
  observer.observe(select, {
    childList: true,
    subtree: true,
    attributes: true,
    attributeFilter: ["disabled", "label"]
  });

  trigger.addEventListener("click", () => {
    if (trigger.disabled) {
      return;
    }
    const willOpen = !wrapper.classList.contains("open");
    closeAllCustomSelects(select);
    wrapper.classList.toggle("open", willOpen);
    trigger.setAttribute("aria-expanded", willOpen ? "true" : "false");
  });

  trigger.addEventListener("keydown", (event) => {
    if (event.key === "ArrowDown" || event.key === "Enter" || event.key === " ") {
      event.preventDefault();
      if (!wrapper.classList.contains("open") && !trigger.disabled) {
        closeAllCustomSelects(select);
        wrapper.classList.add("open");
        trigger.setAttribute("aria-expanded", "true");
      }
    }
  });

  select.addEventListener("change", () => {
    render();
  });

  const controller: CustomSelectController = {
    refresh: () => {
      render();
    },
    dispose: () => {
      observer.disconnect();
      customSelectControllers.delete(select);
      wrapper.replaceWith(select);
      select.classList.remove("custom-select-native");
    }
  };

  customSelectControllers.set(select, controller);
  render();
  return controller;
}

export function refreshEnhancedSelect(select: HTMLSelectElement): void {
  customSelectControllers.get(select)?.refresh();
}
