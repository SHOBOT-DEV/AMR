
export const filterBySearch = (items: any[], searchTerm: string, searchField: string, fieldMap: Record<string, string>) => {
  if (!searchTerm || !searchTerm.trim()) return items;
  const term = searchTerm.trim().toLowerCase();
  
  return items.filter((item) => {
    if (searchField === "any") {
      return Object.values(fieldMap).some((field) =>
        String(item[field] || "").toLowerCase().includes(term)
      );
    }
    const fieldName = fieldMap[searchField];
    return String(item[fieldName] || "").toLowerCase().includes(term);
  });
};